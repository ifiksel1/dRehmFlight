#include "mixer.h"

void mixer::mix(State& s) {
  // Quad mixing - EXAMPLE
  s.motor_scaled[0] = s.thro_des - s.pitch_pid + s.roll_pid + s.yaw_pid; // Front Left
  s.motor_scaled[1] = s.thro_des - s.pitch_pid - s.roll_pid - s.yaw_pid; // Front Right
  s.motor_scaled[2] = s.thro_des + s.pitch_pid - s.roll_pid + s.yaw_pid; // Back Right
  s.motor_scaled[3] = s.thro_des + s.pitch_pid + s.roll_pid - s.yaw_pid; // Back Left
  s.motor_scaled[4] = 0;
  s.motor_scaled[5] = 0;

  s.servo_scaled[0] = 0;
  s.servo_scaled[1] = 0;
  s.servo_scaled[2] = 0;
  s.servo_scaled[3] = 0;
  s.servo_scaled[4] = 0;
  s.servo_scaled[5] = 0;
  s.servo_scaled[6] = 0;
}
