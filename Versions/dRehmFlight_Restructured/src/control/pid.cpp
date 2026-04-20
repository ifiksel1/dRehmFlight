#include "pid.h"
#include "../../config.h"
#include "../storage/config_store.h"
#include <Arduino.h>

// controlANGLE2 outer-loop damping (Beta 1.3 defaults)
static constexpr float B_LOOP_ROLL  = 0.9f;
static constexpr float B_LOOP_PITCH = 0.9f;

// controlANGLE / controlRATE integrator + derivative state
static float integral_roll_prev  = 0.0f;
static float integral_pitch_prev = 0.0f;
static float integral_yaw_prev   = 0.0f;
static float error_roll_prev     = 0.0f;
static float error_pitch_prev    = 0.0f;
static float error_yaw_prev      = 0.0f;
static float GyroX_prev          = 0.0f;
static float GyroY_prev          = 0.0f;

// controlANGLE2 cascade state (separate from single-loop)
static float integral_roll_prev_ol  = 0.0f;
static float integral_pitch_prev_ol = 0.0f;
static float integral_roll_prev_il  = 0.0f;
static float integral_pitch_prev_il = 0.0f;
static float roll_IMU_prev   = 0.0f;
static float pitch_IMU_prev  = 0.0f;
static float roll_des_prev   = 0.0f;
static float pitch_des_prev  = 0.0f;

void pid::reset_integrators() {
  integral_roll_prev = integral_pitch_prev = integral_yaw_prev = 0.0f;
  error_roll_prev = error_pitch_prev = error_yaw_prev = 0.0f;
  GyroX_prev = GyroY_prev = 0.0f;
  integral_roll_prev_ol = integral_pitch_prev_ol = 0.0f;
  integral_roll_prev_il = integral_pitch_prev_il = 0.0f;
  roll_IMU_prev = pitch_IMU_prev = 0.0f;
  roll_des_prev = pitch_des_prev = 0.0f;
}

void pid::get_desired_state(State& s) {
  float thro_des  = (s.rc[0] - 1000.0f) / 1000.0f;
  float roll_des  = (s.rc[1] - 1500.0f) / 500.0f;
  float pitch_des = (s.rc[2] - 1500.0f) / 500.0f;
  float yaw_des   = (s.rc[3] - 1500.0f) / 500.0f;
  float roll_passthru  = roll_des / 2.0f;
  float pitch_passthru = pitch_des / 2.0f;
  float yaw_passthru   = yaw_des / 2.0f;

  thro_des  = constrain(thro_des, 0.0f, 1.0f);
  roll_des  = constrain(roll_des, -1.0f, 1.0f) * MAX_ROLL;
  pitch_des = constrain(pitch_des, -1.0f, 1.0f) * MAX_PITCH;
  yaw_des   = constrain(yaw_des, -1.0f, 1.0f) * MAX_YAW;
  roll_passthru  = constrain(roll_passthru, -0.5f, 0.5f);
  pitch_passthru = constrain(pitch_passthru, -0.5f, 0.5f);
  yaw_passthru   = constrain(yaw_passthru, -0.5f, 0.5f);

  s.thro_des  = thro_des;
  s.roll_des  = roll_des;
  s.pitch_des = pitch_des;
  s.yaw_des   = yaw_des;
  s.roll_passthru  = roll_passthru;
  s.pitch_passthru = pitch_passthru;
  s.yaw_passthru   = yaw_passthru;
}

void pid::control_angle(State& s) {
  const float dt = s.dt;
  const float GyroX = s.gyro.x;
  const float GyroY = s.gyro.y;
  const float GyroZ = s.gyro.z;
  const float roll_IMU  = s.euler.x;
  const float pitch_IMU = s.euler.y;

  // Betaflight-style hard angle limit — clamps setpoint regardless of source
  // (stick, HORIZON blend, auto mode). Pitch/roll only; yaw is always rate.
  const float roll_sp  = constrain(s.roll_des,  -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG);
  const float pitch_sp = constrain(s.pitch_des, -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG);

  // Roll
  float error_roll = roll_sp - roll_IMU;
  float integral_roll = integral_roll_prev + error_roll * dt;
  if (s.rc[0] < 1060) integral_roll = 0;
  integral_roll = constrain(integral_roll, -I_LIMIT, I_LIMIT);
  float derivative_roll = GyroX;
  const auto& c = config_store::data;
  s.roll_pid = 0.01f * (c.kp_roll_angle * error_roll + c.ki_roll_angle * integral_roll - c.kd_roll_angle * derivative_roll);

  // Pitch
  float error_pitch = pitch_sp - pitch_IMU;
  float integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (s.rc[0] < 1060) integral_pitch = 0;
  integral_pitch = constrain(integral_pitch, -I_LIMIT, I_LIMIT);
  float derivative_pitch = GyroY;
  s.pitch_pid = 0.01f * (c.kp_pitch_angle * error_pitch + c.ki_pitch_angle * integral_pitch - c.kd_pitch_angle * derivative_pitch);

  // Yaw (rate)
  float error_yaw = s.yaw_des - GyroZ;
  float integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (s.rc[0] < 1060) integral_yaw = 0;
  integral_yaw = constrain(integral_yaw, -I_LIMIT, I_LIMIT);
  float derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  s.yaw_pid = 0.01f * (c.kp_yaw * error_yaw + c.ki_yaw * integral_yaw + c.kd_yaw * derivative_yaw);

  integral_roll_prev  = integral_roll;
  integral_pitch_prev = integral_pitch;
  error_yaw_prev      = error_yaw;
  integral_yaw_prev   = integral_yaw;
}

void pid::control_angle2(State& s) {
  const float dt = s.dt;
  const float GyroX = s.gyro.x;
  const float GyroY = s.gyro.y;
  const float GyroZ = s.gyro.z;
  const float roll_IMU  = s.euler.x;
  const float pitch_IMU = s.euler.y;

  float roll_des_ol, pitch_des_ol;
  const auto& c = config_store::data;

  // Betaflight-style hard angle limit (see control_angle()).
  const float roll_sp  = constrain(s.roll_des,  -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG);
  const float pitch_sp = constrain(s.pitch_des, -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG);

  // Outer loop - Roll
  float error_roll = roll_sp - roll_IMU;
  float integral_roll_ol = integral_roll_prev_ol + error_roll * dt;
  if (s.rc[0] < 1060) integral_roll_ol = 0;
  integral_roll_ol = constrain(integral_roll_ol, -I_LIMIT, I_LIMIT);
  float derivative_roll = (roll_IMU - roll_IMU_prev) / dt;
  (void)derivative_roll;
  roll_des_ol = c.kp_roll_angle * error_roll + c.ki_roll_angle * integral_roll_ol;

  // Outer loop - Pitch
  float error_pitch = pitch_sp - pitch_IMU;
  float integral_pitch_ol = integral_pitch_prev_ol + error_pitch * dt;
  if (s.rc[0] < 1060) integral_pitch_ol = 0;
  integral_pitch_ol = constrain(integral_pitch_ol, -I_LIMIT, I_LIMIT);
  float derivative_pitch = (pitch_IMU - pitch_IMU_prev) / dt;
  (void)derivative_pitch;
  pitch_des_ol = c.kp_pitch_angle * error_pitch + c.ki_pitch_angle * integral_pitch_ol;

  float Kl = 30.0f;
  roll_des_ol  = Kl * roll_des_ol;
  pitch_des_ol = Kl * pitch_des_ol;
  roll_des_ol  = constrain(roll_des_ol, -240.0f, 240.0f);
  pitch_des_ol = constrain(pitch_des_ol, -240.0f, 240.0f);
  roll_des_ol  = (1.0f - B_LOOP_ROLL)  * roll_des_prev  + B_LOOP_ROLL  * roll_des_ol;
  pitch_des_ol = (1.0f - B_LOOP_PITCH) * pitch_des_prev + B_LOOP_PITCH * pitch_des_ol;

  // Inner loop - Roll
  error_roll = roll_des_ol - GyroX;
  float integral_roll_il = integral_roll_prev_il + error_roll * dt;
  if (s.rc[0] < 1060) integral_roll_il = 0;
  integral_roll_il = constrain(integral_roll_il, -I_LIMIT, I_LIMIT);
  derivative_roll = (error_roll - error_roll_prev) / dt;
  s.roll_pid = 0.01f * (c.kp_roll_rate * error_roll + c.ki_roll_rate * integral_roll_il + c.kd_roll_rate * derivative_roll);

  // Inner loop - Pitch
  error_pitch = pitch_des_ol - GyroY;
  float integral_pitch_il = integral_pitch_prev_il + error_pitch * dt;
  if (s.rc[0] < 1060) integral_pitch_il = 0;
  integral_pitch_il = constrain(integral_pitch_il, -I_LIMIT, I_LIMIT);
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  s.pitch_pid = 0.01f * (c.kp_pitch_rate * error_pitch + c.ki_pitch_rate * integral_pitch_il + c.kd_pitch_rate * derivative_pitch);

  // Yaw
  float error_yaw = s.yaw_des - GyroZ;
  float integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (s.rc[0] < 1060) integral_yaw = 0;
  integral_yaw = constrain(integral_yaw, -I_LIMIT, I_LIMIT);
  float derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  s.yaw_pid = 0.01f * (c.kp_yaw * error_yaw + c.ki_yaw * integral_yaw + c.kd_yaw * derivative_yaw);

  integral_roll_prev_ol  = integral_roll_ol;
  integral_roll_prev_il  = integral_roll_il;
  error_roll_prev        = error_roll;
  roll_IMU_prev          = roll_IMU;
  roll_des_prev          = roll_des_ol;
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev       = error_pitch;
  pitch_IMU_prev         = pitch_IMU;
  pitch_des_prev         = pitch_des_ol;
  error_yaw_prev         = error_yaw;
  integral_yaw_prev      = integral_yaw;
}

void pid::control_rate(State& s) {
  const float dt = s.dt;
  const float GyroX = s.gyro.x;
  const float GyroY = s.gyro.y;
  const float GyroZ = s.gyro.z;
  const auto& c = config_store::data;

  // Roll
  float error_roll = s.roll_des - GyroX;
  float integral_roll = integral_roll_prev + error_roll * dt;
  if (s.rc[0] < 1060) integral_roll = 0;
  integral_roll = constrain(integral_roll, -I_LIMIT, I_LIMIT);
  float derivative_roll = (error_roll - error_roll_prev) / dt;
  s.roll_pid = 0.01f * (c.kp_roll_rate * error_roll + c.ki_roll_rate * integral_roll + c.kd_roll_rate * derivative_roll);

  // Pitch
  float error_pitch = s.pitch_des - GyroY;
  float integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (s.rc[0] < 1060) integral_pitch = 0;
  integral_pitch = constrain(integral_pitch, -I_LIMIT, I_LIMIT);
  float derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  s.pitch_pid = 0.01f * (c.kp_pitch_rate * error_pitch + c.ki_pitch_rate * integral_pitch + c.kd_pitch_rate * derivative_pitch);

  // Yaw
  float error_yaw = s.yaw_des - GyroZ;
  float integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (s.rc[0] < 1060) integral_yaw = 0;
  integral_yaw = constrain(integral_yaw, -I_LIMIT, I_LIMIT);
  float derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  s.yaw_pid = 0.01f * (c.kp_yaw * error_yaw + c.ki_yaw * integral_yaw + c.kd_yaw * derivative_yaw);

  error_roll_prev     = error_roll;
  integral_roll_prev  = integral_roll;
  GyroX_prev          = GyroX;
  error_pitch_prev    = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev          = GyroY;
  error_yaw_prev      = error_yaw;
  integral_yaw_prev   = integral_yaw;
}
