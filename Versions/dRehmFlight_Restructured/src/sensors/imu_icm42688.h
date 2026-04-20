#pragma once

#include "imu.h"
#include "../../config.h"
#include "../ICM42688/ICM42688.h"   // vendored driver

class ImuIcm42688 : public IMU {
 public:
  bool begin() override;
  void readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) override;

 private:
  ICM42688 dev_{SPI, pins::IMU_CS};
};
