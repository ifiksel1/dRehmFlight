#include "../../config.h"
#if defined(IMU_ICM20948) || defined(IMU_MULTI)
#include "imu_icm20948.h"
#include <SPI.h>

// ASSUMES: SparkFun ICM_20948 library — adjust if using a different driver

bool ImuIcm20948::begin() {
  SPI.begin();

  if (dev_.begin(SPI, pins::IMU_CS) != ICM_20948_Stat_Ok) return false;
  if (dev_.swReset() != ICM_20948_Stat_Ok) return false;
  delay(10);
  dev_.sleep(false);
  dev_.lowPower(false);

  ICM_20948_fss_t fss;
  switch (GYRO_FS_DPS) {
    case 250:  fss.g = dps250;  gyro_lsb_per_dps_ = 131.0f; break;
    case 500:  fss.g = dps500;  gyro_lsb_per_dps_ =  65.5f; break;
    case 1000: fss.g = dps1000; gyro_lsb_per_dps_ =  32.8f; break;
    case 2000: fss.g = dps2000; gyro_lsb_per_dps_ =  16.4f; break;
    default:   return false;
  }
  switch (ACCEL_FS_G) {
    case 2:  fss.a = gpm2;  accel_lsb_per_g_ = 16384.0f; break;
    case 4:  fss.a = gpm4;  accel_lsb_per_g_ =  8192.0f; break;
    case 8:  fss.a = gpm8;  accel_lsb_per_g_ =  4096.0f; break;
    case 16: fss.a = gpm16; accel_lsb_per_g_ =  2048.0f; break;
    default: return false;
  }
  if (dev_.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss) != ICM_20948_Stat_Ok) return false;

  // DLPF off — software filtering only (see IMU::update)
  // FIXME: if SparkFun enum names differ, substitute the widest acc/gyr DLPF cfg available.
  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = acc_d473bw_n499bw;
  dlpcfg.g = gyr_d361bw4_n376bw5;
  dev_.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpcfg);
  dev_.enableDLPF(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, false);

  if (dev_.startupMagnetometer() != ICM_20948_Stat_Ok) return false;
  return true;
}

void ImuIcm20948::readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) {
  dev_.getAGMT();

  accel.x = dev_.agmt.acc.axes.x / accel_lsb_per_g_;
  accel.y = dev_.agmt.acc.axes.y / accel_lsb_per_g_;
  accel.z = dev_.agmt.acc.axes.z / accel_lsb_per_g_;

  gyro.x = dev_.agmt.gyr.axes.x / gyro_lsb_per_dps_;
  gyro.y = dev_.agmt.gyr.axes.y / gyro_lsb_per_dps_;
  gyro.z = dev_.agmt.gyr.axes.z / gyro_lsb_per_dps_;

  // AK09916 reports 0.15 uT/LSB
  mag.x = dev_.agmt.mag.axes.x * 0.15f;
  mag.y = dev_.agmt.mag.axes.y * 0.15f;
  mag.z = dev_.agmt.mag.axes.z * 0.15f;
}
#endif
