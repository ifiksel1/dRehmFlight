#include "led.h"
#include "../../config.h"
#include <Arduino.h>

namespace led {

static Pattern current_ = Pattern::OFF;
static uint32_t step_end_ms_ = 0;
static uint8_t  step_        = 0;
static bool     state_       = false;   // current HIGH/LOW

// Per-pattern step sequence: {level, duration_ms}. Loops.
struct Step { bool on; uint16_t ms; };
static const Step OFF_SEQ[]    = {{false, 1000}};
static const Step SOLID_SEQ[]  = {{true,  1000}};
static const Step SLOW_SEQ[]   = {{true,  500}, {false, 500}};
static const Step FAST_SEQ[]   = {{true,  100}, {false, 100}};
static const Step DOUBLE_SEQ[] = {{true, 80}, {false, 80}, {true, 80}, {false, 760}};

static const Step* const SEQS[] = { OFF_SEQ, SOLID_SEQ, SLOW_SEQ, FAST_SEQ, DOUBLE_SEQ };
static const uint8_t SEQ_LEN[]  = { 1, 1, 2, 2, 4 };

static inline void write(bool on) {
#if !defined(BOARD_TEENSY41_DEFAULT)
  if (pins::LED < 0) return;
#endif
  digitalWriteFast(pins::LED, on ? HIGH : LOW);
  state_ = on;
}

void begin() {
#if !defined(BOARD_TEENSY41_DEFAULT)
  if (pins::LED < 0) return;
#endif
  pinMode(pins::LED, OUTPUT);
  write(false);
}

void play(Pattern p) {
  if (p == current_) return;
  current_ = p;
  step_    = 0;
  step_end_ms_ = 0;
}

void update() {
  const uint32_t now = millis();
  if ((int32_t)(now - step_end_ms_) < 0) return;

  const uint8_t idx = (uint8_t)current_;
  const Step* seq   = SEQS[idx];
  const uint8_t len = SEQ_LEN[idx];

  if (step_ >= len) step_ = 0;
  const Step& s = seq[step_];
  write(s.on);
  step_end_ms_ = now + s.ms;
  step_++;
}

}  // namespace led
