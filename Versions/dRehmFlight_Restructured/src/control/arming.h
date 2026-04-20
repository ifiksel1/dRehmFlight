#pragma once

#include "../../state.h"

// Port of Beta 1.3 armedStatus() + throttleCut() gating.
//
// Entry condition (sticky):   CH5 < 1500 AND CH1 < 1050  -> armed = true
// Active cut condition:       !armed OR CH5 > 1500       -> force motor cut
//
// Once armed, stays armed until power cycle — matches Beta 1.3 behavior.
// The caller is responsible for checking should_cut() and calling
// motors.cut() before motors.write() when it returns true.

namespace arming {
  void begin();                   // wire up safety-button pin (no-op on default board)
  void update(State& s);          // flips s.armed on stick arming sequence
  bool should_cut(const State& s);// true when outputs must be held at zero
}
