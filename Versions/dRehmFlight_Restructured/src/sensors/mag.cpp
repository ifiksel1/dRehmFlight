#include "mag.h"
#include "../../config.h"
#include "../storage/config_store.h"

static inline float lp(float prev, float curr, float beta) {
  return (1.0f - beta) * prev + beta * curr;
}

void Mag::update(State& s) {
  Vec3 m;
  readRaw(m);

  const auto& c = config_store::data;
  m.x = (m.x - c.mag_err[0]) * c.mag_scale[0];
  m.y = (m.y - c.mag_err[1]) * c.mag_scale[1];
  m.z = (m.z - c.mag_err[2]) * c.mag_scale[2];

  s.mag.x = lp(mag_prev_.x, m.x, B_MAG);
  s.mag.y = lp(mag_prev_.y, m.y, B_MAG);
  s.mag.z = lp(mag_prev_.z, m.z, B_MAG);
  mag_prev_ = s.mag;
  s.mag_valid = true;
}
