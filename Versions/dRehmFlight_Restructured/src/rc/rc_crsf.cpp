#include "rc_crsf.h"
#include "../../config.h"   // CRSF_SERIAL macro
#include <Arduino.h>
#include <math.h>

static uint8_t crc8_table_[256];
static bool    crc8_ready_ = false;

// CRC-8 polynomial 0xD5, init 0, no reflection
static void crc8_init() {
  for (int i = 0; i < 256; ++i) {
    uint8_t c = (uint8_t)i;
    for (int j = 0; j < 8; ++j) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1);
    crc8_table_[i] = c;
  }
  crc8_ready_ = true;
}

static uint8_t crc8(const uint8_t* p, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; ++i) c = crc8_table_[c ^ p[i]];
  return c;
}

bool RcCrsf::begin() {
  if (!crc8_ready_) crc8_init();
  CRSF_SERIAL.begin(420000);
  for (uint8_t i = 0; i < 16; ++i) last_[i] = 1500;
  last_[0] = 988;
  delay(20);
  return true;
}

bool RcCrsf::parse_frame() {
  if (buf_len_ < 2) return false;
  uint8_t len = buf_[1];
  if (len < 2 || len > 62) { buf_len_ = 0; return false; }
  if (buf_len_ < (uint8_t)(len + 2)) return false;

  uint8_t type = buf_[2];
  uint8_t want_crc = buf_[len + 1];
  uint8_t got_crc = crc8(&buf_[2], len - 1);
  if (want_crc != got_crc) {
    // resync: discard sync and scan for next 0xC8
    uint8_t i = 1;
    while (i < buf_len_ && buf_[i] != 0xC8) ++i;
    uint8_t r = buf_len_ - i;
    for (uint8_t k = 0; k < r; ++k) buf_[k] = buf_[i + k];
    buf_len_ = r;
    return true;
  }

  if (type == 0x16 && len == 24) {
    const uint8_t* p = &buf_[3];
    uint16_t ch[16];
    uint32_t bits = 0;
    uint8_t  nbits = 0;
    uint8_t  idx = 0;
    for (uint8_t i = 0; i < 22 && idx < 16; ++i) {
      bits |= ((uint32_t)p[i]) << nbits;
      nbits += 8;
      while (nbits >= 11 && idx < 16) {
        uint16_t v = (uint16_t)(bits & 0x7FF);
        bits >>= 11;
        nbits -= 11;
        // Betaflight conversion: us = crsf * 1024 / 1639 + 881 → 172→988, 992→1500, 1811→2012
        ch[idx++] = (uint16_t)(((uint32_t)v * 1024) / 1639 + 881);
      }
    }
    for (uint8_t i = 0; i < 16; ++i) last_[i] = ch[i];
    last_frame_ms_ = millis();
  }

  uint8_t consumed = len + 2;
  uint8_t r = buf_len_ - consumed;
  for (uint8_t k = 0; k < r; ++k) buf_[k] = buf_[consumed + k];
  buf_len_ = r;
  return true;
}

void RcCrsf::poll() {
  while (CRSF_SERIAL.available()) {
    uint8_t b = (uint8_t)CRSF_SERIAL.read();
    if (buf_len_ == 0) {
      if (b != 0xC8) continue;
    }
    if (buf_len_ < sizeof(buf_)) {
      buf_[buf_len_++] = b;
    } else {
      buf_len_ = 0;
    }
    while (parse_frame()) {}
  }
}

void RcCrsf::readRaw(uint16_t channels[16], uint8_t& num_channels) {
  poll();
  for (uint8_t i = 0; i < 16; ++i) channels[i] = last_[i];
  num_channels = 16;
}

void RcCrsf::send_frame(uint8_t type, const uint8_t* payload, uint8_t payload_len) {
  uint8_t f[64];
  f[0] = 0xC8;
  f[1] = (uint8_t)(payload_len + 2);
  f[2] = type;
  for (uint8_t i = 0; i < payload_len; ++i) f[3 + i] = payload[i];
  f[3 + payload_len] = crc8(&f[2], payload_len + 1);
  CRSF_SERIAL.write(f, payload_len + 4);
}

static inline void be16(uint8_t* p, int16_t v) {
  p[0] = (uint8_t)((uint16_t)v >> 8);
  p[1] = (uint8_t)((uint16_t)v & 0xFF);
}

void RcCrsf::send_telemetry(const State& s) {
  const float DEG2RAD_E4 = (float)M_PI / 180.0f * 10000.0f;
  uint8_t att[6];
  be16(&att[0], (int16_t)(s.euler.y * DEG2RAD_E4));  // pitch
  be16(&att[2], (int16_t)(s.euler.x * DEG2RAD_E4));  // roll
  be16(&att[4], (int16_t)(s.euler.z * DEG2RAD_E4));  // yaw
  send_frame(0x1E, att, 6);

  if (s.baro_valid) {
    uint8_t baro[4];
    uint16_t alt_dm = (uint16_t)((int32_t)(s.altitude_m * 10.0f) + 0x8000);
    baro[0] = (uint8_t)(alt_dm >> 8);
    baro[1] = (uint8_t)(alt_dm & 0xFF);
    baro[2] = 0;
    baro[3] = 0;
    send_frame(0x09, baro, 4);
  }

  if (s.battery_valid) {
    // CRSF battery sensor frame (0x08): voltage u16 0.1V BE, current u16 0.1A BE,
    // capacity_used u24 mAh BE, percent_remaining u8. Total 8 bytes.
    uint8_t bat[8];
    uint16_t v_dv = (uint16_t)(s.vbat_v * 10.0f);
    uint16_t i_da = (uint16_t)(s.current_a * 10.0f);
    uint32_t mah  = (uint32_t)s.mah_consumed;
    float v_min = 3.3f * (float)BATTERY_CELL_COUNT;
    float v_max = 4.2f * (float)BATTERY_CELL_COUNT;
    float pct_f = 100.0f * (s.vbat_v - v_min) / (v_max - v_min);
    if (pct_f < 0.0f) pct_f = 0.0f; if (pct_f > 100.0f) pct_f = 100.0f;
    bat[0] = (uint8_t)(v_dv >> 8);
    bat[1] = (uint8_t)(v_dv & 0xFF);
    bat[2] = (uint8_t)(i_da >> 8);
    bat[3] = (uint8_t)(i_da & 0xFF);
    bat[4] = (uint8_t)((mah >> 16) & 0xFF);
    bat[5] = (uint8_t)((mah >>  8) & 0xFF);
    bat[6] = (uint8_t)(mah & 0xFF);
    bat[7] = (uint8_t)pct_f;
    send_frame(0x08, bat, 8);
  }
}
