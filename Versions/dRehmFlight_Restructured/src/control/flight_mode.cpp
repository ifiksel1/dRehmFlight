#include "flight_mode.h"
#include "pid.h"
#include "../../config.h"
#include <math.h>

namespace flight_mode {

void update(State& s) {
  const uint16_t ch6 = s.rc[5];
  if      (ch6 < 1300) s.mode = State::FlightMode::ACRO;
  else if (ch6 > 1700) s.mode = State::FlightMode::HORIZON;
  else                 s.mode = State::FlightMode::ANGLE;
}

void dispatch(State& s) {
  switch (s.mode) {
    case State::FlightMode::ACRO:
      pid::control_rate(s);
      return;

    case State::FlightMode::ANGLE:
      pid::control_angle(s);
      return;

    case State::FlightMode::HORIZON: {
      // Run both controllers; blend by max stick deflection.
      // Simplified vs Betaflight — both loops use the same MAX_ROLL/PITCH
      // setpoint scale, so tune MAX_ROLL as your HORIZON max angle and expect
      // pure-rate behavior only at the stick ends.
      pid::control_angle(s);
      const float ang_r = s.roll_pid, ang_p = s.pitch_pid, ang_y = s.yaw_pid;

      pid::control_rate(s);
      const float stick = fmaxf(fabsf(s.roll_des), fabsf(s.pitch_des)) / MAX_ROLL;
      const float w = fminf(fmaxf(stick, 0.0f), 1.0f);   // 0 center, 1 full stick

      s.roll_pid  = (1.0f - w) * ang_r + w * s.roll_pid;
      s.pitch_pid = (1.0f - w) * ang_p + w * s.pitch_pid;
      s.yaw_pid   = (1.0f - w) * ang_y + w * s.yaw_pid;
      return;
    }
  }
}

}  // namespace flight_mode
