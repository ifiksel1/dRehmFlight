#pragma once
#include "../../state.h"

namespace msp {
  void begin();
  void poll(State& s);   // call from scheduler task at ~100 Hz
}
