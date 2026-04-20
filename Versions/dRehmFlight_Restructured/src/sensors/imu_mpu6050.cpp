#include "imu_mpu6050.h"
#include "../../config.h"
#include <Wire.h>

bool ImuMpu6050::begin() {
  Wire.begin();
  Wire.setClock(1000000);   // 1 MHz — above spec but works on Teensy + GY-521

  dev_.initialize();
  if (!dev_.testConnection()) return false;

  uint8_t gyro_range, accel_range;
  switch (GYRO_FS_DPS) {
    case 250:  gyro_range = MPU6050_GYRO_FS_250;  gyro_lsb_per_dps_ = 131.0f; break;
    case 500:  gyro_range = MPU6050_GYRO_FS_500;  gyro_lsb_per_dps_ =  65.5f; break;
    case 1000: gyro_range = MPU6050_GYRO_FS_1000; gyro_lsb_per_dps_ =  32.8f; break;
    case 2000: gyro_range = MPU6050_GYRO_FS_2000; gyro_lsb_per_dps_ =  16.4f; break;
    default:   return false;
  }
  switch (ACCEL_FS_G) {
    case 2:  accel_range = MPU6050_ACCEL_FS_2;  accel_lsb_per_g_ = 16384.0f; break;
    case 4:  accel_range = MPU6050_ACCEL_FS_4;  accel_lsb_per_g_ =  8192.0f; break;
    case 8:  accel_range = MPU6050_ACCEL_FS_8;  accel_lsb_per_g_ =  4096.0f; break;
    case 16: accel_range = MPU6050_ACCEL_FS_16; accel_lsb_per_g_ =  2048.0f; break;
    default: return false;
  }
  dev_.setFullScaleGyroRange(gyro_range);
  dev_.setFullScaleAccelRange(accel_range);
  return true;
}

void ImuMpu6050::readRaw(Vec3& gyro, Vec3& accel, Vec3& /*mag*/) {
  int16_t ax, ay, az, gx, gy, gz;
  dev_.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accel.x = ax / accel_lsb_per_g_;
  accel.y = ay / accel_lsb_per_g_;
  accel.z = az / accel_lsb_per_g_;

  gyro.x = gx / gyro_lsb_per_dps_;
  gyro.y = gy / gyro_lsb_per_dps_;
  gyro.z = gz / gyro_lsb_per_dps_;
}
