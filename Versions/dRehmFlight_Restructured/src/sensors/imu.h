#pragma once

#include "../../state.h"

// Abstract IMU driver.
//
// Subclasses implement begin() and readRaw(). The base class applies the shared
// calibration + LP-filter pipeline in update() so every chip gets identical
// post-processing. Mag output is only valid if hasMag() returns true; the
// standalone mag driver fills State::mag otherwise.

class IMU {
 public:
  virtual ~IMU() = default;

  // Bring up the bus and configure full-scale ranges. Returns false on failure.
  virtual bool begin() = 0;

  // True if this chip has an integrated magnetometer (e.g. MPU9250, ICM-20948).
  virtual bool hasMag() const { return false; }

  // Read raw scaled samples from the chip. Units: gyro deg/s, accel g, mag uT.
  // Implementations must NOT apply calibration or filtering — update() does that.
  virtual void readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) = 0;

  // Two-stage pipeline — call update_calibrated() first, then insert stages
  // that need pre-LP gyro (e.g. RPM notch filter), then call update_filtered().
  // update() is a convenience that runs both back-to-back for callers that
  // don't need to interleave anything.
  void update_calibrated(State& s);   // readRaw -> subtract cal offsets -> State
  void update_filtered(State& s);     // read State -> LP filter -> State
  void update(State& s);              // = update_calibrated + update_filtered

  // Average raw samples at startup and print copy-pasteable error constants.
  // Vehicle must be level on a flat surface. Z-accel has 1 g subtracted.
  void calibrate(uint32_t samples = 12000);

 private:
  Vec3 gyro_prev_{};
  Vec3 accel_prev_{};
  Vec3 mag_prev_{};
};
