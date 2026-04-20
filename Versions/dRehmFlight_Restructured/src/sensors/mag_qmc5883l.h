#pragma once

#include "mag.h"
#include "../QMC5883L/QMC5883L.h"   // vendored driver

class MagQmc5883l : public Mag {
 public:
  bool begin() override;
  void readRaw(Vec3& mag) override;

 private:
  QMC5883L dev_;
};
