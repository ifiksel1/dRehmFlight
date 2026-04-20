#include "../../config.h"
#if defined(MAG_QMC5883L)
#include "mag_qmc5883l.h"
#include <Wire.h>

// ASSUMES: mprograms/QMC5883LCompass-style API (begin/setMode/read) — adjust if using a different driver

bool MagQmc5883l::begin() {
  Wire.begin();
  dev_.begin();
  dev_.setMode(0x01, 0x0C, 0x00, 0x00);   // continuous, 200Hz, ±2G, OSR 512
  return true;
}

void MagQmc5883l::readRaw(Vec3& mag) {
  int16_t x, y, z;
  dev_.read(&x, &y, &z);

  mag.x = x / 1.2f;
  mag.y = y / 1.2f;
  mag.z = z / 1.2f;
}
#endif
