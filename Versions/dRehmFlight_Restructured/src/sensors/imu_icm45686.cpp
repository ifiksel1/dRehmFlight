#include "../../config.h"
#if defined(IMU_ICM45686) || (defined(IMU_MULTI) && defined(BOARD_FMU_RT1062))
#include "imu_icm45686.h"
#include "imu_spi_bus.h"
#include <SPI.h>

// Uses TDK's ICM45686 Arduino library (class ICM456xx): startAccel/startGyro
// take the output data rate in Hz and the full-scale range in g / dps.
//
// The chip's sample rate must be at least CONTROL_RATE_HZ; 3200 Hz leaves
// margin at the default 1500 Hz loop. Raise to 6400 for faster loops.
static constexpr uint16_t ICM45686_ODR_HZ = 3200;

static_assert(ICM45686_ODR_HZ >= CONTROL_RATE_HZ, "ICM-45686 ODR below the control rate");

bool ImuIcm45686::begin() {
  imu_spi_deselect_all();
  SPI.begin();

  if (dev_.begin() != 0) return false;
  if (dev_.startAccel(ICM45686_ODR_HZ, ACCEL_FS_G) != 0) return false;
  if (dev_.startGyro(ICM45686_ODR_HZ, GYRO_FS_DPS) != 0) return false;
  delay(50);   // gyro start-up before the first sample is trusted
  return true;
}

void ImuIcm45686::readRaw(Vec3& gyro, Vec3& accel, Vec3& /*mag*/) {
  inv_imu_sensor_data_t d;
  if (dev_.getDataFromRegisters(d) != 0) return;   // keep previous values on a failed read

  // 16-bit signed counts, full scale = configured range.
  constexpr float ACCEL_G_PER_LSB  = static_cast<float>(ACCEL_FS_G)  / 32768.0f;
  constexpr float GYRO_DPS_PER_LSB = static_cast<float>(GYRO_FS_DPS) / 32768.0f;

  // TODO(layout): rotate into the body frame once the module's IMU
  // orientations are fixed; MultiImu assumes all sub-IMUs share one frame.
  accel.x = d.accel_data[0] * ACCEL_G_PER_LSB;
  accel.y = d.accel_data[1] * ACCEL_G_PER_LSB;
  accel.z = d.accel_data[2] * ACCEL_G_PER_LSB;

  gyro.x = d.gyro_data[0] * GYRO_DPS_PER_LSB;
  gyro.y = d.gyro_data[1] * GYRO_DPS_PER_LSB;
  gyro.z = d.gyro_data[2] * GYRO_DPS_PER_LSB;
}
#endif
