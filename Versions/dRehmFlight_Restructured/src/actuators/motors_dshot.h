#pragma once

#include "motors.h"

// Bidirectional DShot driver for Teensy 4.0/4.1.
// Bit-banged, sequential per motor. Telemetry (eRPM) is sampled inline in
// write() immediately after each motor's frame is sent.

class MotorsDShot : public Motors {
 public:
  bool begin() override;
  void scale(State& s) override;
  void write(const State& s) override;
  void arm() override;
  void cut(State& s) override;
  void readTelemetry(State& s) override;

 private:
  // Mutable telemetry capture from write() (const State&). Indexed by motor.
  uint32_t erpm_[6]       = {0};
  bool     telem_valid_[6] = {false};

  static uint16_t buildFrame(uint16_t throttle, bool telemReq);
  void sendFrame(int pin, uint16_t frame) const;
  bool recvTelemetry(int pin, uint32_t& erpm_out) const;
};
