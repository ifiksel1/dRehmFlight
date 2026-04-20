#pragma once

#include <Arduino.h>

// Single source of truth for runtime data shared across modules.
// Every subsystem takes `State&`. No globals.

struct Vec3 {
  float x, y, z;
};

struct Quat {
  float w, x, y, z;
};

struct State {
  // ---- timing -----------------------------------------------------------
  uint32_t t_us;          // current loop timestamp (micros())
  float    dt;            // seconds since previous control iteration

  // ---- IMU --------------------------------------------------------------
  Vec3 gyro;              // deg/s, calibrated + filtered
  Vec3 accel;             // g,     calibrated + filtered
  Vec3 mag;               // uT,    calibrated + filtered (zero if no mag)
  bool mag_valid;         // true if a mag source populated this tick
  Quat q{1, 0, 0, 0};     // attitude from Madgwick
  Vec3 euler;             // deg: x=roll, y=pitch, z=yaw

  // ---- Baro -------------------------------------------------------------
  float pressure_pa;
  float temperature_c;
  float altitude_m;
  bool  baro_valid;

  // ---- Battery ----------------------------------------------------------
  float vbat_v;            // battery voltage (V), LP-filtered
  float current_a;         // instantaneous current draw (A), LP-filtered
  float mah_consumed;      // integrated mAh since boot
  bool  battery_valid;     // true once first sample taken

  // ---- RC ---------------------------------------------------------------
  uint16_t rc[16];        // channel pulse widths in µs
  bool     rc_failsafe;

  // ---- Setpoint (normalized desired state) ------------------------------
  float thro_des;
  float roll_des, pitch_des, yaw_des;
  float roll_passthru, pitch_passthru, yaw_passthru;

  // ---- Controller output ------------------------------------------------
  float roll_pid, pitch_pid, yaw_pid;

  // ---- Actuator commands ------------------------------------------------
  float  motor_scaled[6];     // 0..1
  float  servo_scaled[7];     // 0..1
  uint16_t motor_pwm_us[6];       // OneShot125 only
  uint16_t servo_pwm_us[7];
  uint16_t motor_dshot[6];        // DShot value 48..2047 (0 = disarmed)
  uint32_t motor_erpm[6];         // bidir DShot: raw eRPM from ESC
  bool     motor_telem_valid[6];  // true for the ticks we got a valid response

  // ---- Flight status ----------------------------------------------------
  bool armed;

  enum class FlightMode : uint8_t { ACRO, ANGLE, HORIZON };
  FlightMode mode = FlightMode::ANGLE;
};
