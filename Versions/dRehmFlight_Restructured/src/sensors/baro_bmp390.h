#pragma once

#include "baro.h"
#include "../../config.h"
#include <Wire.h>

// Bosch BMP390 (and BMP388, same register map) over I2C. Register-level
// driver with Bosch's floating-point compensation, no library.

class BaroBmp390 : public Baro {
 public:
  bool begin() override;
  void readRaw(float& pressure_pa, float& temperature_c) override;

 private:
  bool read(uint8_t reg, uint8_t* buf, uint8_t len);
  bool write(uint8_t reg, uint8_t value);

  // Calibration coefficients, already scaled to floating point.
  float t1_ = 0, t2_ = 0, t3_ = 0;
  float p1_ = 0, p2_ = 0, p3_ = 0, p4_ = 0, p5_ = 0, p6_ = 0;
  float p7_ = 0, p8_ = 0, p9_ = 0, p10_ = 0, p11_ = 0;

  float last_pressure_pa_   = 101325.0f;
  float last_temperature_c_ = 25.0f;
};
