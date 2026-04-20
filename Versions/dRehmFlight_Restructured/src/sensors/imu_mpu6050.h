#pragma once

#include "imu.h"
#include "../MPU6050/MPU6050.h"   // vendored driver (copy from Beta 1.3 src/)

class ImuMpu6050 : public IMU {
 public:
  bool begin() override;
  void readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) override;

 private:
  MPU6050 dev_;
  float gyro_lsb_per_dps_;   // scale factor chosen by GYRO_FS_DPS
  float accel_lsb_per_g_;    // scale factor chosen by ACCEL_FS_G
};
