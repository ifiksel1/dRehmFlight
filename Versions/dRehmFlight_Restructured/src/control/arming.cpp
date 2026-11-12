#include "arming.h"
#include "../../config.h"
#include <Arduino.h>

namespace arming {

// Hardware safety button: active-low, pulled up internally. Pressed = safe.
// Only meaningful on boards that wire pins::SAFETY_BTN (Spin 1 custom board).
static bool safety_pressed() {
#if defined(BOARD_TEENSY41_DEFAULT)
  return false;
#else
  if (pins::SAFETY_BTN < 0) return false;   // board has no safety button on a Teensy pin
  return digitalReadFast(pins::SAFETY_BTN) == LOW;
#endif
}

void begin() {
#if !defined(BOARD_TEENSY41_DEFAULT)
  if (pins::SAFETY_BTN >= 0) pinMode(pins::SAFETY_BTN, INPUT_PULLUP);
#endif
}

void update(State& s) {
  if (!s.armed && s.rc[4] < 1500 && s.rc[0] < 1050 && !safety_pressed()) {
    s.armed = true;
  }
}

bool should_cut(const State& s) {
  return !s.armed || s.rc[4] > 1500 || s.rc_failsafe || safety_pressed();
}

}  // namespace arming
