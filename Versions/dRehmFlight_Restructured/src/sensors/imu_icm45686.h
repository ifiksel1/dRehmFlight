#pragma once

#include "imu.h"
#include "../../config.h"
#include <ICM45686.h>   // TDK "ICM45686" Arduino library (Library Manager); class ICM456xx

class ImuIcm45686 : public IMU {
 public:
  explicit ImuIcm45686(uint8_t cs_pin) : dev_(SPI, cs_pin) {}

  bool begin() override;
  void readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) override;

 private:
  ICM456xx dev_;
};
