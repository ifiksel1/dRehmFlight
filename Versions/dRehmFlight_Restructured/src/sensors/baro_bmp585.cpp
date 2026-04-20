#include "../../config.h"
#if defined(BARO_BMP585)
#include "baro_bmp585.h"
#include <Wire.h>

// ASSUMES: SparkFun BMP581/BMP585 Arduino library — adjust if using a different driver.

bool BaroBmp585::begin() {
  Wire.begin();
  if (dev_.beginI2C(0x47) != 0) return false;
  return true;
}

void BaroBmp585::readRaw(float& pressure_pa, float& temperature_c) {
  bmp5_sensor_data data{};
  dev_.getSensorData(&data);
  pressure_pa   = data.pressure;
  temperature_c = data.temperature;
}
#endif
