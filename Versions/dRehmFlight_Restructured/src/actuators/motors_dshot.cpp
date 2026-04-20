#include "motors_dshot.h"
#include "../../config.h"
#include <Arduino.h>

// Teensy 4 cycle counter: 600 MHz -> 600 cycles/us.
static constexpr uint32_t F_CPU_MHZ   = 600;
static constexpr uint32_t BIT_CYCLES  = (F_CPU_MHZ * 1000) / DSHOT_KBPS;          // full bit period
static constexpr uint32_t T1H_CYCLES  = (BIT_CYCLES * 3) / 4;                     // '1' high portion
static constexpr uint32_t T0H_CYCLES  = BIT_CYCLES / 4;                           // '0' high portion
// Bidir telemetry response runs at 5/4 the DShot bitrate.
static constexpr uint32_t RESP_BIT_CYCLES = (BIT_CYCLES * 4) / 5;

static inline uint32_t cyc() { return ARM_DWT_CYCCNT; }
static inline void wait_until(uint32_t deadline) { while ((int32_t)(cyc() - deadline) < 0) {} }

// Standard DShot/BLHeli bidir GCR 5b->4b decode. 0xFF = invalid quintet.
static constexpr uint8_t GCR_LUT[32] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,
  0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07,
  0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF,
};

uint16_t MotorsDShot::buildFrame(uint16_t throttle, bool telemReq) {
  uint16_t data = (throttle << 1) | (telemReq ? 1 : 0);
  // Bidir DShot: CRC is the inverted nibble XOR.
  uint16_t crc = (~(data ^ (data >> 4) ^ (data >> 8))) & 0x0F;
  return (data << 4) | crc;
}

void MotorsDShot::sendFrame(int pin, uint16_t frame) const {
  // Bidir DShot idles HIGH; each bit drops LOW for 1/4 (bit=1) or 3/4 (bit=0)
  // of the period. We express that by holding HIGH for T1H/T0H then LOW for
  // the remainder of the bit.
  noInterrupts();
  uint32_t t = cyc();
  for (int i = 15; i >= 0; i--) {
    uint32_t high_cyc = (frame & (1u << i)) ? T1H_CYCLES : T0H_CYCLES;
    digitalWriteFast(pin, HIGH);
    wait_until(t + high_cyc);
    digitalWriteFast(pin, LOW);
    t += BIT_CYCLES;
    wait_until(t);
  }
  digitalWriteFast(pin, HIGH);  // return to idle
  interrupts();
}

bool MotorsDShot::recvTelemetry(int pin, uint32_t& erpm_out) const {
  pinMode(pin, INPUT_PULLUP);

  // Wait for ESC to pull line low (start of response). Budget ~40us.
  uint32_t start_deadline = cyc() + F_CPU_MHZ * 40;
  while (digitalReadFast(pin)) {
    if ((int32_t)(cyc() - start_deadline) >= 0) { pinMode(pin, OUTPUT); digitalWriteFast(pin, HIGH); return false; }
  }

  // Sample 21 bits at the response bit period, centered in each bit cell.
  noInterrupts();
  uint32_t t = cyc() + RESP_BIT_CYCLES / 2;
  uint32_t raw = 0;
  for (int i = 0; i < 21; i++) {
    wait_until(t);
    raw = (raw << 1) | (digitalReadFast(pin) ? 1u : 0u);
    t += RESP_BIT_CYCLES;
  }
  interrupts();

  pinMode(pin, OUTPUT);
  digitalWriteFast(pin, HIGH);

  // Bidir DShot line code: stream is the XOR-decoded GCR-21. First, undo
  // the running-XOR: out[i] = in[i] ^ in[i-1], MSB-first with implicit in[-1]=0.
  uint32_t g = 0;
  uint32_t prev = 0;
  for (int i = 20; i >= 0; i--) {
    uint32_t bit = (raw >> i) & 1u;
    uint32_t out = bit ^ prev;
    g = (g << 1) | out;
    prev = bit;
  }
  // g is now 21 bits, but only 20 are meaningful (leading bit is the implicit start).
  // Actually bidir DShot GCR stream is 20 bits of data -> 16 bits after decode.
  // Slice into 4 quintets (MSB first) and map via GCR_LUT.
  uint16_t dec = 0;
  for (int i = 3; i >= 0; i--) {
    uint8_t q = (g >> (i * 5)) & 0x1F;
    uint8_t n = GCR_LUT[q];
    if (n == 0xFF) return false;
    dec = (dec << 4) | n;
  }

  // Verify CRC: lower 4 bits are CRC over upper 12 bits, same inverted formula.
  uint16_t payload = dec >> 4;
  uint16_t crc_rx  = dec & 0x0F;
  uint16_t crc_calc = (~(payload ^ (payload >> 4) ^ (payload >> 8))) & 0x0F;
  if (crc_rx != crc_calc) return false;

  // payload = [4-bit exponent][8-bit mantissa] -> eRPM period in us.
  uint8_t  exp  = (payload >> 8) & 0x0F;
  uint8_t  mant =  payload       & 0xFF;
  uint32_t period_us = (uint32_t)mant << exp;
  if (period_us == 0) return false;

  erpm_out = 60u * 1000000u / period_us;
  return true;
}

bool MotorsDShot::begin() {
  // Enable cycle counter once. ARM_DEMCR / ARM_DWT_CTRL are set up already on
  // Teensy 4 in most cores, but doing it here is idempotent.
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  for (int i = 0; i < 6; i++) {
    pinMode(pins::M[i], OUTPUT);
    digitalWriteFast(pins::M[i], HIGH);  // bidir idle
    telem_valid_[i] = false;
    erpm_[i] = 0;
  }
  delay(10);
  return true;
}

void MotorsDShot::scale(State& s) {
  for (int i = 0; i < 6; i++) {
    int v = (int)(s.motor_scaled[i] * 1999.0f) + 48;
    if (v < 48)   v = 48;
    if (v > 2047) v = 2047;
    s.motor_dshot[i] = (uint16_t)v;
  }
}

void MotorsDShot::write(const State& s) {
  // Sequential per motor: send frame, then sample telemetry response on the
  // same pin. ~85us per motor * 6 = ~510us worst case, but telem recv() bails
  // out after ~40us if no response, so realistic total well under the 500us
  // control budget. We sample inline here (rather than in readTelemetry())
  // because the ESC's response window opens ~30us after the frame ends — if
  // we returned to the scheduler between, we'd miss it.
  for (int i = 0; i < 6; i++) {
    uint16_t frame = buildFrame(s.motor_dshot[i], /*telemReq=*/true);
    sendFrame(pins::M[i], frame);

    uint32_t erpm = 0;
    if (recvTelemetry(pins::M[i], erpm)) {
      erpm_[i] = erpm;
      telem_valid_[i] = true;
    } else {
      telem_valid_[i] = false;
    }
  }
}

void MotorsDShot::readTelemetry(State& s) {
  // Telemetry is captured inline in write() (see comment there). Here we only
  // publish the last-captured values into State.
  for (int i = 0; i < 6; i++) {
    s.motor_telem_valid[i] = telem_valid_[i];
    if (telem_valid_[i]) {
      // erpm_ is electrical RPM. Mechanical RPM = eRPM / POLE_PAIRS, but we
      // store raw eRPM as documented in state.h.
      s.motor_erpm[i] = erpm_[i];
    }
  }
}

void MotorsDShot::arm() {
  // Hold zero-throttle armed frames for ~500ms so the ESC sees a valid signal
  // and arms. BLHeli_32 / AM32 ESCs must be preconfigured for bidirectional
  // DShot via their config tool — we do not send the "set bidir mode" command
  // frame here.
  State s{};
  for (int i = 0; i < 6; i++) s.motor_dshot[i] = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    write(s);
    delayMicroseconds(500);
  }
}

void MotorsDShot::cut(State& s) {
  for (int i = 0; i < 6; i++) {
    s.motor_scaled[i] = 0.0f;
    s.motor_dshot[i]  = 0;   // DShot 0 = disarmed / stop
  }
}
