#pragma once

#include "../../state.h"

namespace battery {
  void begin();
  void update(State& s);
  bool is_low(const State& s);
  bool is_critical(const State& s);
}
