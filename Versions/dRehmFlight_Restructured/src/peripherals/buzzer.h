#pragma once

#include "../../state.h"

namespace buzzer {

enum class Pattern : uint8_t {
  NONE,
  ARMED,             // short ack beep on arm edge
  DISARMED,          // double beep on disarm edge
  ARM_REFUSED,       // long low buzz when arm conditions not met
  LOW_BATTERY,       // repeating mid-pitch every 1s
  CRITICAL_BATTERY,  // fast chirps
  FAILSAFE,          // alternating tones
  BEACON             // lost-model finder
};

void begin();
void play(Pattern p);   // starts pattern; no-op if already playing that pattern
void update();          // call from ~50 Hz scheduler task; advances state machine

}  // namespace buzzer
