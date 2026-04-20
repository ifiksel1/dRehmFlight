#include "baro.h"
#include <math.h>

void Baro::update(State& s) {
  float p, t;
  readRaw(p, t);

  const float p0 = 101325.0f;
  s.pressure_pa   = p;
  s.temperature_c = t;
  s.altitude_m    = 44330.0f * (1.0f - powf(p / p0, 1.0f / 5.255f));
  s.baro_valid    = true;
}
