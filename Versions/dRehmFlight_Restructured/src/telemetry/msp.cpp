#include "msp.h"
#include "../../config.h"
#include "../storage/config_store.h"
#include <Arduino.h>

namespace msp {

// MSP v1 cmd IDs
static constexpr uint8_t MSP_API_VERSION   = 1;
static constexpr uint8_t MSP_FC_VARIANT    = 2;
static constexpr uint8_t MSP_FC_VERSION    = 3;
static constexpr uint8_t MSP_BOARD_INFO    = 4;
static constexpr uint8_t MSP_BUILD_INFO    = 5;
static constexpr uint8_t MSP_NAME          = 10;
static constexpr uint8_t MSP_STATUS        = 101;
static constexpr uint8_t MSP_RAW_IMU       = 102;
static constexpr uint8_t MSP_SERVO         = 103;
static constexpr uint8_t MSP_MOTOR         = 104;
static constexpr uint8_t MSP_RC            = 105;
static constexpr uint8_t MSP_ATTITUDE      = 108;
static constexpr uint8_t MSP_ALTITUDE      = 109;
static constexpr uint8_t MSP_ANALOG        = 110;
static constexpr uint8_t MSP_RC_TUNING     = 111;
static constexpr uint8_t MSP_PID           = 112;
static constexpr uint8_t MSP_STATUS_EX     = 150;
static constexpr uint8_t MSP_SET_PID       = 202;
static constexpr uint8_t MSP_SET_RC_TUNING = 204;
static constexpr uint8_t MSP_EEPROM_WRITE  = 250;

// ---- CRC-8 DVB-S2 (poly 0xD5), same polynomial as CRSF ----------------------
static uint8_t crc8_table_[256];
static bool    crc8_ready_ = false;

static void crc8_init() {
  for (int i = 0; i < 256; ++i) {
    uint8_t c = (uint8_t)i;
    for (int j = 0; j < 8; ++j) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1);
    crc8_table_[i] = c;
  }
  crc8_ready_ = true;
}

static uint8_t crc8_dvbs2(uint8_t seed, const uint8_t* p, uint16_t n) {
  uint8_t c = seed;
  for (uint16_t i = 0; i < n; ++i) c = crc8_table_[c ^ p[i]];
  return c;
}

// ---- parser state -----------------------------------------------------------
enum PState : uint8_t {
  P_IDLE, P_M_OR_X, P_DIR,
  P_V1_LEN, P_V1_CMD, P_V1_PAYLOAD, P_V1_CKSUM,
  P_V2_FLAG, P_V2_CMD_LO, P_V2_CMD_HI, P_V2_LEN_LO, P_V2_LEN_HI, P_V2_PAYLOAD, P_V2_CRC,
};

static PState   st_        = P_IDLE;
static bool     is_v2_     = false;
static uint16_t rx_cmd_    = 0;
static uint16_t rx_len_    = 0;
static uint16_t rx_idx_    = 0;
static uint8_t  rx_cksum_  = 0;
static uint8_t  rx_buf_[256];

static uint8_t  tx_buf_[256];
static uint32_t cycle_us_ = 0;

// ---- TX helpers -------------------------------------------------------------
static void send_v1(uint8_t cmd, const uint8_t* payload, uint8_t len) {
  tx_buf_[0] = '$';
  tx_buf_[1] = 'M';
  tx_buf_[2] = '>';
  tx_buf_[3] = len;
  tx_buf_[4] = cmd;
  for (uint8_t i = 0; i < len; ++i) tx_buf_[5 + i] = payload[i];
  uint8_t c = len ^ cmd;
  for (uint8_t i = 0; i < len; ++i) c ^= payload[i];
  tx_buf_[5 + len] = c;
  Serial.write(tx_buf_, 6 + len);
}

static void send_v2(uint16_t cmd, const uint8_t* payload, uint16_t len) {
  tx_buf_[0] = '$';
  tx_buf_[1] = 'X';
  tx_buf_[2] = '>';
  tx_buf_[3] = 0;
  tx_buf_[4] = (uint8_t)(cmd & 0xFF);
  tx_buf_[5] = (uint8_t)(cmd >> 8);
  tx_buf_[6] = (uint8_t)(len & 0xFF);
  tx_buf_[7] = (uint8_t)(len >> 8);
  for (uint16_t i = 0; i < len; ++i) tx_buf_[8 + i] = payload[i];
  uint8_t c = crc8_dvbs2(0, &tx_buf_[3], 5 + len);
  tx_buf_[8 + len] = c;
  Serial.write(tx_buf_, 9 + len);
}

static inline void reply(uint16_t cmd, const uint8_t* payload, uint16_t len) {
  if (is_v2_ || cmd >= 256) send_v2(cmd, payload, len);
  else                      send_v1((uint8_t)cmd, payload, (uint8_t)len);
}

// ---- payload packing helpers -----------------------------------------------
static inline void w_u8 (uint8_t*& p, uint8_t  v) { *p++ = v; }
static inline void w_u16(uint8_t*& p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p+=2; }
static inline void w_i16(uint8_t*& p, int16_t  v) { w_u16(p, (uint16_t)v); }
static inline void w_u32(uint8_t*& p, uint32_t v) {
  p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); p+=4;
}
static inline void w_i32(uint8_t*& p, int32_t  v) { w_u32(p, (uint32_t)v); }
static inline void w_str(uint8_t*& p, const char* s) { while (*s) *p++ = (uint8_t)*s++; }

// Betaflight PID byte scaling: P byte = KP * 10 (so KP=0.5 -> 5).
// I byte = KI * 10, D byte = KD * 1000 (D gains are ~0.0002..0.05).
static inline uint8_t pid_byte_p(float v) { int x = (int)(v * 10.0f   + 0.5f); if (x<0)x=0; if (x>255)x=255; return (uint8_t)x; }
static inline uint8_t pid_byte_i(float v) { int x = (int)(v * 10.0f   + 0.5f); if (x<0)x=0; if (x>255)x=255; return (uint8_t)x; }
static inline uint8_t pid_byte_d(float v) { int x = (int)(v * 1000.0f + 0.5f); if (x<0)x=0; if (x>255)x=255; return (uint8_t)x; }
static inline float   pid_from_p(uint8_t b) { return (float)b / 10.0f; }
static inline float   pid_from_i(uint8_t b) { return (float)b / 10.0f; }
static inline float   pid_from_d(uint8_t b) { return (float)b / 1000.0f; }

// ---- command dispatch -------------------------------------------------------
static void handle(uint16_t cmd, const uint8_t* in, uint16_t in_len, State& s) {
  uint8_t  pl[256];
  uint8_t* p = pl;

  switch (cmd) {
    case MSP_API_VERSION: {
      w_u8(p, 0); w_u8(p, 1); w_u8(p, 45);
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_FC_VARIANT: {
      // Pretend to be Betaflight so Configurator connects.
      w_str(p, "BTFL");
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_FC_VERSION: {
      w_u8(p, 4); w_u8(p, 4); w_u8(p, 0);
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_BOARD_INFO: {
      w_str(p, "DRFL");                    // board identifier (4 chars)
      w_u16(p, 0);                         // hw revision
      w_u8(p, 0);                          // fc type
      w_u8(p, 0);                          // target capabilities
      const char* name = "DREHMFLIGHT";
      w_u8(p, 11);
      w_str(p, name);
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_BUILD_INFO: {
      const char* d = __DATE__;            // "Mmm dd yyyy" - 11 chars
      const char* t = __TIME__;            // "hh:mm:ss"    -  8 chars
      for (uint8_t i = 0; i < 11; ++i) w_u8(p, (uint8_t)d[i]);
      for (uint8_t i = 0; i <  8; ++i) w_u8(p, (uint8_t)t[i]);
      for (uint8_t i = 0; i <  7; ++i) w_u8(p, (uint8_t)' ');   // git sha placeholder
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_NAME: {
      w_str(p, "dRehmFlight");
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_STATUS:
    case MSP_STATUS_EX: {
      w_u16(p, (uint16_t)cycle_us_);       // cycle time us
      w_u16(p, 0);                         // i2c errors
      w_u16(p, 0x0001);                    // sensor flags: ACC present
      uint32_t mode_flags = 0;
      if (s.armed) mode_flags |= 0x01;
      if (s.mode == State::FlightMode::ANGLE)   mode_flags |= 0x02;
      if (s.mode == State::FlightMode::HORIZON) mode_flags |= 0x04;
      w_u32(p, mode_flags);
      w_u8(p, 0);                          // current profile
      w_u16(p, 0);                         // cpuload
      if (cmd == MSP_STATUS_EX) {
        w_u8(p, 1);                        // max profile count
        w_u8(p, 0);                        // rate profile index
        w_u8(p, 0);                        // additional flag count
        // arming disable flags (count above = 0, so nothing follows)
        w_u8(p, s.armed ? 0 : 1);          // arming disable flags count placeholder
        w_u32(p, s.armed ? 0u : 1u);       // arming disable flags
        w_u8(p, 0);                        // config state flag
      }
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_RAW_IMU: {
      // Accel in ~512 LSB/g, gyro in ~16.4 LSB/dps, mag int16 (0 if absent).
      w_i16(p, (int16_t)(s.accel.x * 512.0f));
      w_i16(p, (int16_t)(s.accel.y * 512.0f));
      w_i16(p, (int16_t)(s.accel.z * 512.0f));
      w_i16(p, (int16_t)(s.gyro.x  * 16.4f));
      w_i16(p, (int16_t)(s.gyro.y  * 16.4f));
      w_i16(p, (int16_t)(s.gyro.z  * 16.4f));
      if (s.mag_valid) {
        w_i16(p, (int16_t)s.mag.x);
        w_i16(p, (int16_t)s.mag.y);
        w_i16(p, (int16_t)s.mag.z);
      } else {
        w_i16(p, 0); w_i16(p, 0); w_i16(p, 0);
      }
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_ATTITUDE: {
      w_i16(p, (int16_t)(s.euler.x * 10.0f));   // roll  deg*10
      w_i16(p, (int16_t)(s.euler.y * 10.0f));   // pitch deg*10
      w_i16(p, (int16_t)(s.euler.z));           // yaw   deg
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_ALTITUDE: {
      w_i32(p, (int32_t)(s.altitude_m * 100.0f));   // cm
      w_i16(p, 0);                                  // vario cm/s (not tracked)
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_ANALOG: {
      w_u8(p, (uint8_t)(s.vbat_v * 10.0f));          // vbat 0.1V (0..25.5V covers up to 6S)
      w_i16(p, (int16_t)s.mah_consumed);             // mAh drawn
      w_u16(p, 0);                                   // rssi (no CRSF link-stat yet)
      w_i16(p, (int16_t)(s.current_a * 100.0f));     // amperage 0.01A
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_RC: {
      for (uint8_t i = 0; i < 16; ++i) w_u16(p, s.rc[i]);
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_MOTOR: {
      for (uint8_t i = 0; i < 8; ++i) {
        float v = (i < 6) ? s.motor_scaled[i] : 0.0f;
        if (v < 0.0f) v = 0.0f; if (v > 1.0f) v = 1.0f;
        w_u16(p, (uint16_t)(1000.0f + v * 1000.0f));
      }
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_SERVO: {
      for (uint8_t i = 0; i < 8; ++i) {
        float v = (i < 7) ? s.servo_scaled[i] : 0.0f;
        if (v < 0.0f) v = 0.0f; if (v > 1.0f) v = 1.0f;
        w_u16(p, (uint16_t)(1000.0f + v * 1000.0f));
      }
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_PID: {
      // 10 × [P, I, D]. Betaflight order: ROLL, PITCH, YAW, ALT, POS, POSR,
      // NAVR, LEVEL, MAG, VEL. Only ROLL/PITCH/YAW/LEVEL populated.
      // Scaling: P,I byte = gain*10; D byte = gain*1000.
      auto& g = config_store::data;
      w_u8(p, pid_byte_p(g.kp_roll_rate));   w_u8(p, pid_byte_i(g.ki_roll_rate));   w_u8(p, pid_byte_d(g.kd_roll_rate));
      w_u8(p, pid_byte_p(g.kp_pitch_rate));  w_u8(p, pid_byte_i(g.ki_pitch_rate));  w_u8(p, pid_byte_d(g.kd_pitch_rate));
      w_u8(p, pid_byte_p(g.kp_yaw));         w_u8(p, pid_byte_i(g.ki_yaw));         w_u8(p, pid_byte_d(g.kd_yaw));
      for (uint8_t i = 0; i < 4; ++i) { w_u8(p, 0); w_u8(p, 0); w_u8(p, 0); }        // ALT, POS, POSR, NAVR
      w_u8(p, pid_byte_p(g.kp_roll_angle));  w_u8(p, pid_byte_i(g.ki_roll_angle));  w_u8(p, pid_byte_d(g.kd_roll_angle));
      for (uint8_t i = 0; i < 2; ++i) { w_u8(p, 0); w_u8(p, 0); w_u8(p, 0); }        // MAG, VEL
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_SET_PID: {
      // Write-through: live edits affect runtime immediately; persistence
      // happens only on explicit MSP_EEPROM_WRITE (Betaflight convention).
      if (in_len >= 30) {
        auto& g = config_store::data;
        g.kp_roll_rate   = pid_from_p(in[0]);  g.ki_roll_rate   = pid_from_i(in[1]);  g.kd_roll_rate   = pid_from_d(in[2]);
        g.kp_pitch_rate  = pid_from_p(in[3]);  g.ki_pitch_rate  = pid_from_i(in[4]);  g.kd_pitch_rate  = pid_from_d(in[5]);
        g.kp_yaw         = pid_from_p(in[6]);  g.ki_yaw         = pid_from_i(in[7]);  g.kd_yaw         = pid_from_d(in[8]);
        // indexes 9..20: ALT, POS, POSR, NAVR — ignored
        g.kp_roll_angle  = pid_from_p(in[21]); g.ki_roll_angle  = pid_from_i(in[22]); g.kd_roll_angle  = pid_from_d(in[23]);
        g.kp_pitch_angle = g.kp_roll_angle;    // Betaflight shares LEVEL across axes
        g.ki_pitch_angle = g.ki_roll_angle;
        g.kd_pitch_angle = g.kd_roll_angle;
      }
      reply(cmd, pl, 0);
    } break;

    case MSP_RC_TUNING: {
      // FIXME: rates curves are task #26 — reply with zeros for now.
      for (uint8_t i = 0; i < 11; ++i) w_u8(p, 0);
      reply(cmd, pl, (uint16_t)(p - pl));
    } break;

    case MSP_SET_RC_TUNING: {
      // no-op ack
      reply(cmd, pl, 0);
    } break;

    case MSP_EEPROM_WRITE: {
      config_store::save();
      reply(cmd, pl, 0);
    } break;

    default:
      // unknown: send empty response so Configurator doesn't stall
      reply(cmd, pl, 0);
      break;
  }
}

// ---- parser -----------------------------------------------------------------
static void feed(uint8_t b, State& s) {
  switch (st_) {
    case P_IDLE:
      if (b == '$') st_ = P_M_OR_X;
      break;
    case P_M_OR_X:
      if      (b == 'M') { is_v2_ = false; st_ = P_DIR; }
      else if (b == 'X') { is_v2_ = true;  st_ = P_DIR; }
      else               { st_ = (b == '$') ? P_M_OR_X : P_IDLE; }
      break;
    case P_DIR:
      if (b == '<') st_ = is_v2_ ? P_V2_FLAG : P_V1_LEN;
      else          st_ = P_IDLE;
      break;

    // v1
    case P_V1_LEN:
      rx_len_ = b; rx_cksum_ = b; rx_idx_ = 0;
      if (rx_len_ > sizeof(rx_buf_)) { st_ = P_IDLE; break; }
      st_ = P_V1_CMD;
      break;
    case P_V1_CMD:
      rx_cmd_ = b; rx_cksum_ ^= b;
      st_ = rx_len_ ? P_V1_PAYLOAD : P_V1_CKSUM;
      break;
    case P_V1_PAYLOAD:
      rx_buf_[rx_idx_++] = b; rx_cksum_ ^= b;
      if (rx_idx_ >= rx_len_) st_ = P_V1_CKSUM;
      break;
    case P_V1_CKSUM:
      if (b == rx_cksum_) handle(rx_cmd_, rx_buf_, rx_len_, s);
      st_ = P_IDLE;
      break;

    // v2
    case P_V2_FLAG:
      rx_buf_[0] = b;                      // stash flag for CRC
      rx_cksum_ = crc8_dvbs2(0, &b, 1);
      st_ = P_V2_CMD_LO;
      break;
    case P_V2_CMD_LO:
      rx_cmd_ = b; rx_cksum_ = crc8_dvbs2(rx_cksum_, &b, 1);
      st_ = P_V2_CMD_HI;
      break;
    case P_V2_CMD_HI:
      rx_cmd_ |= ((uint16_t)b << 8); rx_cksum_ = crc8_dvbs2(rx_cksum_, &b, 1);
      st_ = P_V2_LEN_LO;
      break;
    case P_V2_LEN_LO:
      rx_len_ = b; rx_cksum_ = crc8_dvbs2(rx_cksum_, &b, 1);
      st_ = P_V2_LEN_HI;
      break;
    case P_V2_LEN_HI:
      rx_len_ |= ((uint16_t)b << 8); rx_cksum_ = crc8_dvbs2(rx_cksum_, &b, 1);
      rx_idx_ = 0;
      if (rx_len_ > sizeof(rx_buf_)) { st_ = P_IDLE; break; }
      st_ = rx_len_ ? P_V2_PAYLOAD : P_V2_CRC;
      break;
    case P_V2_PAYLOAD:
      rx_buf_[rx_idx_++] = b; rx_cksum_ = crc8_dvbs2(rx_cksum_, &b, 1);
      if (rx_idx_ >= rx_len_) st_ = P_V2_CRC;
      break;
    case P_V2_CRC:
      if (b == rx_cksum_) handle(rx_cmd_, rx_buf_, rx_len_, s);
      st_ = P_IDLE;
      break;
  }
}

// ---- public -----------------------------------------------------------------
void begin() {
  if (!crc8_ready_) crc8_init();
  Serial.begin(115200);
}

void poll(State& s) {
  cycle_us_ = (uint32_t)(s.dt * 1e6f);
  while (Serial.available()) {
    feed((uint8_t)Serial.read(), s);
  }
}

}  // namespace msp
