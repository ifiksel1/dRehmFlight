// THE airframe customization point. Map roll_pid/pitch_pid/yaw_pid/thro_des to
// motor_scaled[] and servo_scaled[]. Replace the body below for your airframe.
#pragma once

#include "../../state.h"

namespace mixer {
  void mix(State& s);
}
