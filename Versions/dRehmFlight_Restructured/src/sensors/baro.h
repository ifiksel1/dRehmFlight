#pragma once

#include "../../state.h"

// Abstract barometer driver.
//
// Subclasses implement begin() and readRaw(). The base class converts pressure
// to altitude via the standard-atmosphere approximation in update() so every
// chip gets identical post-processing.

class Baro {
 public:
  virtual ~Baro() = default;

  // Bring up the bus and configure the sensor. Returns false on failure.
  virtual bool begin() = 0;

  // Read raw scaled samples from the chip. Units: pressure Pa, temperature °C.
  // Implementations must NOT touch State — update() does that.
  virtual void readRaw(float& pressure_pa, float& temperature_c) = 0;

  // Read + convert to altitude, writing into State.
  void update(State& s);
};
