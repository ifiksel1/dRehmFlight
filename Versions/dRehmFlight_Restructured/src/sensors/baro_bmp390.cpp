#include "../../config.h"
#if defined(BARO_BMP390)
#include "baro_bmp390.h"

// Register map and compensation formulas are from the Bosch BMP390 datasheet
// (BST-BMP390-DS002, section 8 and appendix). Written from the datasheet, not
// yet run on hardware.

#ifndef BARO_WIRE
#define BARO_WIRE Wire
#endif

namespace {

constexpr uint8_t I2C_ADDR     = 0x77;   // SDO high
constexpr uint8_t REG_CHIP_ID  = 0x00;   // 0x60 = BMP390, 0x50 = BMP388
constexpr uint8_t REG_DATA     = 0x04;   // pressure (3 bytes) then temperature (3 bytes), LSB first
constexpr uint8_t REG_PWR_CTRL = 0x1B;
constexpr uint8_t REG_OSR      = 0x1C;
constexpr uint8_t REG_ODR      = 0x1D;
constexpr uint8_t REG_CONFIG   = 0x1F;
constexpr uint8_t REG_CALIB    = 0x31;   // 21 bytes
constexpr uint8_t REG_CMD      = 0x7E;

constexpr uint8_t CMD_SOFTRESET       = 0xB6;
constexpr uint8_t PWR_NORMAL_PT_ON    = 0x33;   // normal mode, pressure + temperature enabled
constexpr uint8_t OSR_P_X8_T_X1       = 0x03;   // pressure x8, temperature x1
constexpr uint8_t ODR_50HZ            = 0x02;   // 200 Hz / 2^2; x8 oversampling fits in 20 ms
constexpr uint8_t IIR_COEF_3          = 0x02 << 1;

}  // namespace

bool BaroBmp390::read(uint8_t reg, uint8_t* buf, uint8_t len) {
  BARO_WIRE.beginTransmission(I2C_ADDR);
  BARO_WIRE.write(reg);
  if (BARO_WIRE.endTransmission(false) != 0) return false;
  if (BARO_WIRE.requestFrom(I2C_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = BARO_WIRE.read();
  return true;
}

bool BaroBmp390::write(uint8_t reg, uint8_t value) {
  BARO_WIRE.beginTransmission(I2C_ADDR);
  BARO_WIRE.write(reg);
  BARO_WIRE.write(value);
  return BARO_WIRE.endTransmission() == 0;
}

bool BaroBmp390::begin() {
  BARO_WIRE.begin();
  BARO_WIRE.setClock(400000);

  uint8_t id = 0;
  if (!read(REG_CHIP_ID, &id, 1)) return false;
  if (id != 0x60 && id != 0x50) return false;

  if (!write(REG_CMD, CMD_SOFTRESET)) return false;
  delay(10);

  uint8_t c[21];
  if (!read(REG_CALIB, c, sizeof(c))) return false;

  auto u16 = [&](uint8_t i) { return static_cast<uint16_t>(c[i] | (c[i + 1] << 8)); };
  auto s16 = [&](uint8_t i) { return static_cast<int16_t>(u16(i)); };
  auto s8  = [&](uint8_t i) { return static_cast<int8_t>(c[i]); };

  // Scaling per the datasheet's "calibration coefficient" table.
  t1_  = u16(0)  * 256.0f;                         // / 2^-8
  t2_  = u16(2)  / 1073741824.0f;                  // / 2^30
  t3_  = s8(4)   / 281474976710656.0f;             // / 2^48
  p1_  = (s16(5) - 16384.0f) / 1048576.0f;         // (x - 2^14) / 2^20
  p2_  = (s16(7) - 16384.0f) / 536870912.0f;       // (x - 2^14) / 2^29
  p3_  = s8(9)   / 4294967296.0f;                  // / 2^32
  p4_  = s8(10)  / 137438953472.0f;                // / 2^37
  p5_  = u16(11) * 8.0f;                           // / 2^-3
  p6_  = u16(13) / 64.0f;                          // / 2^6
  p7_  = s8(15)  / 256.0f;                         // / 2^8
  p8_  = s8(16)  / 32768.0f;                       // / 2^15
  p9_  = s16(17) / 281474976710656.0f;             // / 2^48
  p10_ = s8(19)  / 281474976710656.0f;             // / 2^48
  p11_ = s8(20)  / 36893488147419103232.0f;        // / 2^65

  if (!write(REG_OSR, OSR_P_X8_T_X1)) return false;
  if (!write(REG_ODR, ODR_50HZ)) return false;
  if (!write(REG_CONFIG, IIR_COEF_3)) return false;
  if (!write(REG_PWR_CTRL, PWR_NORMAL_PT_ON)) return false;
  delay(40);
  return true;
}

void BaroBmp390::readRaw(float& pressure_pa, float& temperature_c) {
  uint8_t d[6];
  if (read(REG_DATA, d, sizeof(d))) {
    const float up = static_cast<float>(static_cast<uint32_t>(d[0]) | (static_cast<uint32_t>(d[1]) << 8) |
                                        (static_cast<uint32_t>(d[2]) << 16));
    const float ut = static_cast<float>(static_cast<uint32_t>(d[3]) | (static_cast<uint32_t>(d[4]) << 8) |
                                        (static_cast<uint32_t>(d[5]) << 16));

    // Temperature
    const float td = ut - t1_;
    const float t  = td * t2_ + td * td * t3_;

    // Pressure
    const float out1 = p5_ + p6_ * t + p7_ * t * t + p8_ * t * t * t;
    const float out2 = up * (p1_ + p2_ * t + p3_ * t * t + p4_ * t * t * t);
    const float out3 = up * up * (p9_ + p10_ * t) + up * up * up * p11_;

    last_temperature_c_ = t;
    last_pressure_pa_   = out1 + out2 + out3;
  }
  // On a failed read the previous sample is returned.
  pressure_pa   = last_pressure_pa_;
  temperature_c = last_temperature_c_;
}
#endif
