#include "../../config.h"
#if defined(IMU_ICM42688) || defined(IMU_MULTI)
#include "imu_icm42688.h"
#include <SPI.h>

// ASSUMES: Bolder Flight ICM42688 library — adjust if using a different driver

bool ImuIcm42688::begin() {
  SPI.begin();

  if (dev_.begin() < 0) return false;

  ICM42688::GyroFS gyro_range;
  switch (GYRO_FS_DPS) {
    case 250:  gyro_range = ICM42688::dps250;  break;
    case 500:  gyro_range = ICM42688::dps500;  break;
    case 1000: gyro_range = ICM42688::dps1000; break;
    case 2000: gyro_range = ICM42688::dps2000; break;
    default:   return false;
  }
  ICM42688::AccelFS accel_range;
  switch (ACCEL_FS_G) {
    case 2:  accel_range = ICM42688::gpm2;  break;
    case 4:  accel_range = ICM42688::gpm4;  break;
    case 8:  accel_range = ICM42688::gpm8;  break;
    case 16: accel_range = ICM42688::gpm16; break;
    default: return false;
  }
  if (dev_.setGyroFS(gyro_range)  < 0) return false;
  if (dev_.setAccelFS(accel_range) < 0) return false;
  // DLPF off — software filtering only (see IMU::update)
  // FIXME: Bolder Flight ICM42688 setFilters() disables the UI filter but the AAF is not
  // reachable through the lib — poke GYRO_CONFIG_STATIC3..5 / ACCEL_CONFIG_STATIC2..4 if needed.
  if (dev_.setFilters(false, false) < 0) return false;
  return true;
}

void ImuIcm42688::readRaw(Vec3& gyro, Vec3& accel, Vec3& /*mag*/) {
  dev_.getAGT();

  constexpr float MSS_TO_G    = 1.0f / 9.80665f;
  constexpr float RADPS_TO_DPS = 57.2957795f;

  accel.x = dev_.accX() * MSS_TO_G;
  accel.y = dev_.accY() * MSS_TO_G;
  accel.z = dev_.accZ() * MSS_TO_G;

  gyro.x = dev_.gyrX() * RADPS_TO_DPS;
  gyro.y = dev_.gyrY() * RADPS_TO_DPS;
  gyro.z = dev_.gyrZ() * RADPS_TO_DPS;
}
#endif
