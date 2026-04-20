#pragma once

#include "../../state.h"

// Betaflight-style RPM notch filter. Cascades 3 biquad notches per motor per
// gyro axis (fundamental + 2nd + 3rd harmonic) at the motor's mechanical
// rotation frequency derived from bidirectional-DShot eRPM telemetry.

namespace rpm_filter {
  void init();
  void apply(State& s);
}
