#include "battery.h"
#include "../../config.h"
#include <Arduino.h>

namespace battery {

static uint32_t low_since_us_ = 0;   // critical-timer: 0 == not currently below cutoff

void begin() {
  analogReadResolution(12);
  pinMode(pins::VBAT_ADC, INPUT);
  pinMode(pins::ESC_CURRENT_ADC, INPUT);
  low_since_us_ = 0;
}

void update(State& s) {
  int   v_raw = analogRead(pins::VBAT_ADC);
  float v     = ((float)v_raw / 4095.0f) * 3.3f * VBAT_DIVIDER_RATIO;

  int   i_raw = analogRead(pins::ESC_CURRENT_ADC);
  float v_i   = ((float)i_raw / 4095.0f) * 3.3f;
  float a     = (v_i - ESC_CURRENT_OFFSET_V) / ESC_CURRENT_V_PER_A;

  if (!s.battery_valid) {
    s.vbat_v       = v;
    s.current_a    = a;
    s.mah_consumed = 0.0f;
    s.battery_valid = true;
  } else {
    s.vbat_v    += VBAT_FILTER_BETA          * (v - s.vbat_v);
    s.current_a += ESC_CURRENT_FILTER_BETA   * (a - s.current_a);
  }

  s.mah_consumed += s.current_a * s.dt * 1000.0f / 3600.0f;
}

bool is_low(const State& s) {
  if (!s.battery_valid) return false;
  float per_cell = s.vbat_v / (float)BATTERY_CELL_COUNT;
  return per_cell < VBAT_WARN_V;
}

bool is_critical(const State& s) {
  if (!s.battery_valid) return false;
  float per_cell = s.vbat_v / (float)BATTERY_CELL_COUNT;
  // Arm timer on first dip; require 1 second sustained so transient sag doesn't trip.
  uint32_t now = micros();
  if (per_cell < VBAT_CUT_V) {
    if (low_since_us_ == 0) low_since_us_ = now;
    return (now - low_since_us_) > 1000000u;
  }
  low_since_us_ = 0;
  return false;
}

}  // namespace battery
