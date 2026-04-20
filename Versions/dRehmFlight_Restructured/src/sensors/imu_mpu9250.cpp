#include "imu_mpu9250.h"
#include "../../config.h"
#include <SPI.h>

ImuMpu9250::ImuMpu9250() : dev_(SPI2, pins::IMU_CS) {}

bool ImuMpu9250::begin() {
  if (dev_.begin() < 0) return false;

  MPU9250::GyroRange gyro_range;
  MPU9250::AccelRange accel_range;
  switch (GYRO_FS_DPS) {
    case 250:  gyro_range = MPU9250::GYRO_RANGE_250DPS;  gyro_lsb_per_dps_ = 131.0f; break;
    case 500:  gyro_range = MPU9250::GYRO_RANGE_500DPS;  gyro_lsb_per_dps_ =  65.5f; break;
    case 1000: gyro_range = MPU9250::GYRO_RANGE_1000DPS; gyro_lsb_per_dps_ =  32.8f; break;
    case 2000: gyro_range = MPU9250::GYRO_RANGE_2000DPS; gyro_lsb_per_dps_ =  16.4f; break;
    default:   return false;
  }
  switch (ACCEL_FS_G) {
    case 2:  accel_range = MPU9250::ACCEL_RANGE_2G;  accel_lsb_per_g_ = 16384.0f; break;
    case 4:  accel_range = MPU9250::ACCEL_RANGE_4G;  accel_lsb_per_g_ =  8192.0f; break;
    case 8:  accel_range = MPU9250::ACCEL_RANGE_8G;  accel_lsb_per_g_ =  4096.0f; break;
    case 16: accel_range = MPU9250::ACCEL_RANGE_16G; accel_lsb_per_g_ =  2048.0f; break;
    default: return false;
  }
  if (dev_.setGyroRange(gyro_range) < 0) return false;
  if (dev_.setAccelRange(accel_range) < 0) return false;
  // DLPF off — software filtering only (see IMU::update)
  // FIXME: Bolder Flight MPU9250 lib has no DLPF_BANDWIDTH_OFF; 184Hz is the widest non-bypassed setting.
  if (dev_.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ) < 0) return false;

  // Neutralize driver-level mag cal — IMU::update() applies MAG_ERR_*/MAG_SCALE_*.
  dev_.setMagCalX(0.0f, 1.0f);
  dev_.setMagCalY(0.0f, 1.0f);
  dev_.setMagCalZ(0.0f, 1.0f);

  if (dev_.setSrd(0) < 0) return false;   // gyro/accel 1 kHz, mag 100 Hz
  return true;
}

void ImuMpu9250::readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) {
  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
  dev_.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  accel.x = ax / accel_lsb_per_g_;
  accel.y = ay / accel_lsb_per_g_;
  accel.z = az / accel_lsb_per_g_;

  gyro.x = gx / gyro_lsb_per_dps_;
  gyro.y = gy / gyro_lsb_per_dps_;
  gyro.z = gz / gyro_lsb_per_dps_;

  mag.x = mx / 6.0f;
  mag.y = my / 6.0f;
  mag.z = mz / 6.0f;
}
