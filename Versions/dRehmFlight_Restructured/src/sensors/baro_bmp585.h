#pragma once

#include "baro.h"
#include "../BMP585/BMP585.h"   // vendored driver

class BaroBmp585 : public Baro {
 public:
  bool begin() override;
  void readRaw(float& pressure_pa, float& temperature_c) override;

 private:
  BMP585 dev_;
};
