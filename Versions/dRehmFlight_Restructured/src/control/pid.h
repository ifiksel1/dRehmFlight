#pragma once

#include "../../state.h"

namespace pid {
  void get_desired_state(State& s);
  void control_angle(State& s);
  void control_angle2(State& s);
  void control_rate(State& s);
  void reset_integrators();
}
