#pragma once

#include "../../state.h"

// CH6 3-position switch -> FlightMode:
//   < 1300 us : ACRO     (pure rate, no self-level)
//   1300..1700: ANGLE    (bounded tilt, self-level)
//   > 1700 us : HORIZON  (ANGLE blend at center, rate contribution at stick ends)

namespace flight_mode {
  void update(State& s);     // read s.rc[5], write s.mode
  void dispatch(State& s);   // run the right PID controller for the current mode
}
