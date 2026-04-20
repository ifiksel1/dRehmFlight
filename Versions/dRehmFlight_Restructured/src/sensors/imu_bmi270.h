#pragma once

#include "imu.h"
#include "../BMI270/BMI270.h"   // vendored driver

class ImuBmi270 : public IMU {
 public:
  bool begin() override;
  void readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) override;

 private:
  BMI270 dev_;
};
