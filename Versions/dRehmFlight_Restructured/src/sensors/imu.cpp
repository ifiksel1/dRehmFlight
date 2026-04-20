#include "imu.h"
#include "../../config.h"
#include "../storage/config_store.h"
#include <Arduino.h>

static inline float lp(float prev, float curr, float beta) {
  return (1.0f - beta) * prev + beta * curr;
}

void IMU::update_calibrated(State& s) {
  Vec3 g, a, m;
  readRaw(g, a, m);

  const auto& c = config_store::data;

  s.accel.x = a.x - c.acc_err[0];
  s.accel.y = a.y - c.acc_err[1];
  s.accel.z = a.z - c.acc_err[2];

  s.gyro.x = g.x - c.gyro_err[0];
  s.gyro.y = g.y - c.gyro_err[1];
  s.gyro.z = g.z - c.gyro_err[2];

  if (hasMag()) {
    s.mag.x = (m.x - c.mag_err[0]) * c.mag_scale[0];
    s.mag.y = (m.y - c.mag_err[1]) * c.mag_scale[1];
    s.mag.z = (m.z - c.mag_err[2]) * c.mag_scale[2];
    s.mag_valid = true;
  }
}

void IMU::update_filtered(State& s) {
  s.accel.x = lp(accel_prev_.x, s.accel.x, B_ACCEL);
  s.accel.y = lp(accel_prev_.y, s.accel.y, B_ACCEL);
  s.accel.z = lp(accel_prev_.z, s.accel.z, B_ACCEL);
  accel_prev_ = s.accel;

  s.gyro.x = lp(gyro_prev_.x, s.gyro.x, B_GYRO);
  s.gyro.y = lp(gyro_prev_.y, s.gyro.y, B_GYRO);
  s.gyro.z = lp(gyro_prev_.z, s.gyro.z, B_GYRO);
  gyro_prev_ = s.gyro;

  if (hasMag() && s.mag_valid) {
    s.mag.x = lp(mag_prev_.x, s.mag.x, B_MAG);
    s.mag.y = lp(mag_prev_.y, s.mag.y, B_MAG);
    s.mag.z = lp(mag_prev_.z, s.mag.z, B_MAG);
    mag_prev_ = s.mag;
  }
}

void IMU::update(State& s) {
  update_calibrated(s);
  update_filtered(s);
}

void IMU::calibrate(uint32_t samples) {
  Vec3 g, a, m;
  double ax = 0, ay = 0, az = 0;
  double gx = 0, gy = 0, gz = 0;
  double mx = 0, my = 0, mz = 0;

  for (uint32_t i = 0; i < samples; i++) {
    readRaw(g, a, m);
    ax += a.x; ay += a.y; az += a.z;
    gx += g.x; gy += g.y; gz += g.z;
    if (hasMag()) { mx += m.x; my += m.y; mz += m.z; }
  }

  const double n = (double)samples;
  ax /= n; ay /= n; az /= n;
  gx /= n; gy /= n; gz /= n;
  az -= 1.0;  // vehicle level — subtract 1 g

  Serial.println("// Paste these into config.h — vehicle must be level:");
  Serial.print("static constexpr float ACC_ERR_X = ");  Serial.print((float)ax, 6); Serial.println("f;");
  Serial.print("static constexpr float ACC_ERR_Y = ");  Serial.print((float)ay, 6); Serial.println("f;");
  Serial.print("static constexpr float ACC_ERR_Z = ");  Serial.print((float)az, 6); Serial.println("f;");
  Serial.print("static constexpr float GYRO_ERR_X = "); Serial.print((float)gx, 6); Serial.println("f;");
  Serial.print("static constexpr float GYRO_ERR_Y = "); Serial.print((float)gy, 6); Serial.println("f;");
  Serial.print("static constexpr float GYRO_ERR_Z = "); Serial.print((float)gz, 6); Serial.println("f;");

  auto& c = config_store::data;
  c.acc_err[0]  = (float)ax; c.acc_err[1]  = (float)ay; c.acc_err[2]  = (float)az;
  c.gyro_err[0] = (float)gx; c.gyro_err[1] = (float)gy; c.gyro_err[2] = (float)gz;

  if (hasMag()) {
    mx /= n; my /= n; mz /= n;
    Serial.print("static constexpr float MAG_ERR_X = "); Serial.print((float)mx, 6); Serial.println("f;");
    Serial.print("static constexpr float MAG_ERR_Y = "); Serial.print((float)my, 6); Serial.println("f;");
    Serial.print("static constexpr float MAG_ERR_Z = "); Serial.print((float)mz, 6); Serial.println("f;");
    c.mag_err[0] = (float)mx; c.mag_err[1] = (float)my; c.mag_err[2] = (float)mz;
  }

  config_store::save();
}
