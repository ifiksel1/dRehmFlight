#include "servos.h"
#include "../../config.h"

void Servos::begin() {
  for (size_t i = 0; i < SERVO_COUNT; i++) servo_[i].attach(pins::SERVO[i]);
}

void Servos::scale(State& s) {
  for (size_t i = 0; i < SERVO_COUNT; i++) {
    int v = (int)(s.servo_scaled[i] * 180.0f);
    if (v < 0) v = 0;
    if (v > 180) v = 180;
    s.servo_pwm_us[i] = (uint16_t)v;
  }
}

void Servos::write(const State& s) {
  for (size_t i = 0; i < SERVO_COUNT; i++) servo_[i].write(s.servo_pwm_us[i]);
}
