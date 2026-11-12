#include "../../config.h"
#if defined(IMU_BMI088) || (defined(IMU_MULTI) && defined(BOARD_FMU_RT1062))
#include "imu_bmi088.h"
#include "imu_spi_bus.h"

// Register addresses and values are from the Bosch BMI088 datasheet
// (BST-BMI088-DS001). Written from the datasheet, not yet run on hardware.

namespace {

const SPISettings kSpi(10000000, MSBFIRST, SPI_MODE0);

// Accelerometer
constexpr uint8_t ACC_CHIP_ID   = 0x00;   // reads 0x1E
constexpr uint8_t ACC_DATA      = 0x12;   // X LSB .. Z MSB, 6 bytes
constexpr uint8_t ACC_CONF      = 0x40;
constexpr uint8_t ACC_RANGE     = 0x41;
constexpr uint8_t ACC_PWR_CONF  = 0x7C;
constexpr uint8_t ACC_PWR_CTRL  = 0x7D;
constexpr uint8_t ACC_SOFTRESET = 0x7E;
constexpr uint8_t ACC_ID_VALUE  = 0x1E;
constexpr uint8_t ACC_CONF_1600HZ_NORMAL = 0xAC;   // bwp = normal, ODR = 1600 Hz

// Gyro
constexpr uint8_t GYR_CHIP_ID   = 0x00;   // reads 0x0F
constexpr uint8_t GYR_DATA      = 0x02;   // X LSB .. Z MSB, 6 bytes
constexpr uint8_t GYR_RANGE     = 0x0F;
constexpr uint8_t GYR_BANDWIDTH = 0x10;
constexpr uint8_t GYR_LPM1      = 0x11;
constexpr uint8_t GYR_SOFTRESET = 0x14;
constexpr uint8_t GYR_ID_VALUE  = 0x0F;
constexpr uint8_t GYR_BW_2000HZ_532HZ = 0x00;      // ODR 2000 Hz, filter 532 Hz

constexpr uint8_t SOFTRESET_CMD = 0xB6;
constexpr uint8_t READ_FLAG     = 0x80;

}  // namespace

void ImuBmi088::write_reg(uint8_t cs, uint8_t reg, uint8_t value) {
  SPI.beginTransaction(kSpi);
  digitalWriteFast(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWriteFast(cs, HIGH);
  SPI.endTransaction();
}

// The accelerometer sends one dummy byte before the data on every read.
void ImuBmi088::read_accel(uint8_t reg, uint8_t* buf, uint8_t len) {
  SPI.beginTransaction(kSpi);
  digitalWriteFast(cs_accel_, LOW);
  SPI.transfer(reg | READ_FLAG);
  SPI.transfer(0x00);                       // dummy byte
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
  digitalWriteFast(cs_accel_, HIGH);
  SPI.endTransaction();
}

void ImuBmi088::read_gyro(uint8_t reg, uint8_t* buf, uint8_t len) {
  SPI.beginTransaction(kSpi);
  digitalWriteFast(cs_gyro_, LOW);
  SPI.transfer(reg | READ_FLAG);
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
  digitalWriteFast(cs_gyro_, HIGH);
  SPI.endTransaction();
}

uint8_t ImuBmi088::read_accel_reg(uint8_t reg) {
  uint8_t v = 0;
  read_accel(reg, &v, 1);
  return v;
}

uint8_t ImuBmi088::read_gyro_reg(uint8_t reg) {
  uint8_t v = 0;
  read_gyro(reg, &v, 1);
  return v;
}

bool ImuBmi088::begin() {
  imu_spi_deselect_all();
  pinMode(cs_accel_, OUTPUT);
  pinMode(cs_gyro_, OUTPUT);
  digitalWriteFast(cs_accel_, HIGH);
  digitalWriteFast(cs_gyro_, HIGH);
  SPI.begin();

  // ---- accelerometer ----
  // It powers up in I2C mode; the first chip-select edge switches it to SPI,
  // so the first read is discarded. Repeat after the reset.
  (void)read_accel_reg(ACC_CHIP_ID);
  write_reg(cs_accel_, ACC_SOFTRESET, SOFTRESET_CMD);
  delay(2);
  (void)read_accel_reg(ACC_CHIP_ID);
  if (read_accel_reg(ACC_CHIP_ID) != ACC_ID_VALUE) return false;

  uint8_t acc_range;   // full scale = 1.5 g * 2^(range + 1): 3, 6, 12, 24 g
  switch (ACCEL_FS_G) {
    case 2:  acc_range = 0; break;   // nearest BMI088 range: 3 g
    case 4:  acc_range = 1; break;   // 6 g
    case 8:  acc_range = 2; break;   // 12 g
    case 16: acc_range = 3; break;   // 24 g
    default: return false;
  }
  accel_g_per_lsb_ = (1.5f * static_cast<float>(1u << (acc_range + 1))) / 32768.0f;

  write_reg(cs_accel_, ACC_PWR_CONF, 0x00);   // active mode
  delay(1);
  write_reg(cs_accel_, ACC_PWR_CTRL, 0x04);   // accelerometer on
  delay(50);
  write_reg(cs_accel_, ACC_CONF, ACC_CONF_1600HZ_NORMAL);
  write_reg(cs_accel_, ACC_RANGE, acc_range);

  // ---- gyro ----
  write_reg(cs_gyro_, GYR_SOFTRESET, SOFTRESET_CMD);
  delay(30);
  if (read_gyro_reg(GYR_CHIP_ID) != GYR_ID_VALUE) return false;

  uint8_t gyr_range;
  float   gyr_fs_dps;
  switch (GYRO_FS_DPS) {
    case 250:  gyr_range = 3; gyr_fs_dps = 250.0f;  break;
    case 500:  gyr_range = 2; gyr_fs_dps = 500.0f;  break;
    case 1000: gyr_range = 1; gyr_fs_dps = 1000.0f; break;
    case 2000: gyr_range = 0; gyr_fs_dps = 2000.0f; break;
    default:   return false;
  }
  gyro_dps_per_lsb_ = gyr_fs_dps / 32768.0f;

  write_reg(cs_gyro_, GYR_RANGE, gyr_range);
  write_reg(cs_gyro_, GYR_BANDWIDTH, GYR_BW_2000HZ_532HZ);
  write_reg(cs_gyro_, GYR_LPM1, 0x00);        // normal mode
  delay(30);
  return true;
}

void ImuBmi088::readRaw(Vec3& gyro, Vec3& accel, Vec3& /*mag*/) {
  uint8_t a[6];
  uint8_t g[6];
  read_accel(ACC_DATA, a, 6);
  read_gyro(GYR_DATA, g, 6);

  auto s16 = [](uint8_t lsb, uint8_t msb) {
    return static_cast<int16_t>(static_cast<uint16_t>(msb) << 8 | lsb);
  };

  // TODO(layout): rotate into the body frame once the module's IMU
  // orientations are fixed; MultiImu assumes all sub-IMUs share one frame.
  accel.x = s16(a[0], a[1]) * accel_g_per_lsb_;
  accel.y = s16(a[2], a[3]) * accel_g_per_lsb_;
  accel.z = s16(a[4], a[5]) * accel_g_per_lsb_;

  gyro.x = s16(g[0], g[1]) * gyro_dps_per_lsb_;
  gyro.y = s16(g[2], g[3]) * gyro_dps_per_lsb_;
  gyro.z = s16(g[4], g[5]) * gyro_dps_per_lsb_;
}
#endif
