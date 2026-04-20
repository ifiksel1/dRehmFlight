#pragma once

#include "imu.h"
#include <stdint.h>

class MultiImu : public IMU {
 public:
  MultiImu(IMU** sub_imus, uint8_t count);

  bool begin() override;
  bool hasMag() const override;
  void readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) override;

 private:
  IMU** sub_ = nullptr;
  uint8_t count_ = 0;
};
