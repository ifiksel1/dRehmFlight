#pragma once

#include <stdint.h>

namespace led {

enum class Pattern : uint8_t {
  OFF,
  SOLID,         // armed + ok
  SLOW_BLINK,    // disarmed (1 Hz)
  FAST_BLINK,    // critical battery / failsafe (5 Hz)
  DOUBLE_BLINK   // low battery warning
};

void begin();
void play(Pattern p);   // no-op if already playing
void update();          // call from ~50 Hz scheduler task

}  // namespace led
