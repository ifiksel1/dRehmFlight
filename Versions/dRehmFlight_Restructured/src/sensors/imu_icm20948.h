#pragma once

#include "imu.h"
#include "../ICM20948/ICM20948.h"   // vendored driver (SparkFun ICM_20948)

class ImuIcm20948 : public IMU {
 public:
  bool begin() override;
  bool hasMag() const override { return true; }
  void readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) override;

 private:
  ICM_20948_SPI dev_;
  float gyro_lsb_per_dps_;   // scale factor chosen by GYRO_FS_DPS
  float accel_lsb_per_g_;    // scale factor chosen by ACCEL_FS_G
};
