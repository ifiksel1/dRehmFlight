#pragma once

#include "../../state.h"

// Abstract magnetometer driver.
//
// Parallels IMU: subclasses implement begin() and readRaw(); the base class
// applies the shared calibration + LP-filter pipeline in update(), matching
// the mag branch of IMU::update() so a standalone Mag is interchangeable
// with an IMU's onboard mag from the caller's perspective.

class Mag {
 public:
  virtual ~Mag() = default;

  // Bring up the bus and configure the device. Returns false on failure.
  virtual bool begin() = 0;

  // Read raw scaled sample from the chip. Units: uT.
  // Implementations must NOT apply calibration or filtering — update() does that.
  virtual void readRaw(Vec3& mag) = 0;

  // Read + calibrate + LP-filter, writing into State.
  void update(State& s);

 private:
  Vec3 mag_prev_{};
};
