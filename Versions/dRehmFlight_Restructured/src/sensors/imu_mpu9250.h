#pragma once

#include "imu.h"
#include "../MPU9250/MPU9250.h"   // vendored driver (copy from Beta 1.3 src/)

class ImuMpu9250 : public IMU {
 public:
  ImuMpu9250();
  bool hasMag() const override { return true; }
  bool begin() override;
  void readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) override;

 private:
  MPU9250 dev_;
  float gyro_lsb_per_dps_;
  float accel_lsb_per_g_;
};
