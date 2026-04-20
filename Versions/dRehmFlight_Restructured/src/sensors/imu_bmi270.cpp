#include "../../config.h"
#if defined(IMU_BMI270) || defined(IMU_MULTI)
#include "imu_bmi270.h"
#include <SPI.h>

// ASSUMES: Arduino Nicla Sense BMI270 lib — swap signatures if using a different vendor driver.
// That API returns accel in g and gyro in deg/s already, so no LSB scaling is needed here.

bool ImuBmi270::begin() {
  SPI.begin();
  pinMode(pins::IMU_CS, OUTPUT);
  digitalWrite(pins::IMU_CS, HIGH);

  if (!dev_.begin(pins::IMU_CS)) return false;

  uint8_t gyro_range, accel_range;
  switch (GYRO_FS_DPS) {
    case 125:  gyro_range = BMI270_GYRO_RANGE_125;  break;
    case 250:  gyro_range = BMI270_GYRO_RANGE_250;  break;
    case 500:  gyro_range = BMI270_GYRO_RANGE_500;  break;
    case 1000: gyro_range = BMI270_GYRO_RANGE_1000; break;
    case 2000: gyro_range = BMI270_GYRO_RANGE_2000; break;
    default:   return false;
  }
  switch (ACCEL_FS_G) {
    case 2:  accel_range = BMI270_ACCEL_RANGE_2G;  break;
    case 4:  accel_range = BMI270_ACCEL_RANGE_4G;  break;
    case 8:  accel_range = BMI270_ACCEL_RANGE_8G;  break;
    case 16: accel_range = BMI270_ACCEL_RANGE_16G; break;
    default: return false;
  }
  dev_.setGyroRange(gyro_range);
  dev_.setAccelRange(accel_range);
  // DLPF off — software filtering only (see IMU::update)
  // TODO(user): Arduino Nicla BMI270 lib does not expose filter config. Poke ACC_CONF (0x40) and
  // GYR_CONF (0x42) via the lib's register-write backdoor: OSR4 / ODR=1600Hz, filter=0 (no filter).
  return true;
}

void ImuBmi270::readRaw(Vec3& gyro, Vec3& accel, Vec3& /*mag*/) {
  dev_.readAcceleration(accel.x, accel.y, accel.z);
  dev_.readGyroscope(gyro.x, gyro.y, gyro.z);
}
#endif
