#pragma once

#include "imu.h"
#include "../../config.h"
#include <SPI.h>

// Bosch BMI088: accelerometer and gyro are two separate SPI devices in one
// package, each with its own chip select. Register-level driver, no library.

class ImuBmi088 : public IMU {
 public:
  ImuBmi088(uint8_t cs_accel, uint8_t cs_gyro) : cs_accel_(cs_accel), cs_gyro_(cs_gyro) {}

  bool begin() override;
  void readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) override;

 private:
  void    write_reg(uint8_t cs, uint8_t reg, uint8_t value);
  uint8_t read_accel_reg(uint8_t reg);
  uint8_t read_gyro_reg(uint8_t reg);
  void    read_accel(uint8_t reg, uint8_t* buf, uint8_t len);
  void    read_gyro(uint8_t reg, uint8_t* buf, uint8_t len);

  uint8_t cs_accel_;
  uint8_t cs_gyro_;
  float   accel_g_per_lsb_  = 0.0f;
  float   gyro_dps_per_lsb_ = 0.0f;
};
