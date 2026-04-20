#pragma once

#include "../../state.h"

// Abstract motor-output driver. Subclasses implement a specific ESC protocol
// (OneShot125 analog PWM, DShot digital, etc).

class Motors {
 public:
  virtual ~Motors() = default;

  virtual bool begin() = 0;
  virtual void scale(State& s) = 0;           // float scaled[] -> protocol-native values
  virtual void write(const State& s) = 0;     // push latest frame to ESCs
  virtual void arm() = 0;                     // hold minimum output long enough for ESC to arm
  virtual void cut(State& s) = 0;             // force zero output this instant

  // Bi-directional DShot reads back eRPM per motor — no-op for analog protocols.
  virtual void readTelemetry(State& /*s*/) {}

  // One-shot ESC throttle calibration (analog-PWM ESCs only). DShot has its
  // own command-frame mechanism; implementations that don't need this can
  // leave the default no-op.
  virtual void calibrateEscs() {}
};
