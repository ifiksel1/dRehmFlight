#include "motors_oneshot125.h"
#include "../../config.h"
#include <Arduino.h>

bool MotorsOneShot125::begin() {
  for (int i = 0; i < 6; i++) pinMode(pins::M[i], OUTPUT);
  return true;
}

void MotorsOneShot125::scale(State& s) {
  for (int i = 0; i < 6; i++) {
    int v = (int)(s.motor_scaled[i] * 125.0f + 125.0f);
    if (v < 125) v = 125;
    if (v > 250) v = 250;
    s.motor_pwm_us[i] = (uint16_t)v;
  }
}

// OneShot125: simultaneous HIGH on all 6 pins, poll micros() and drop each
// pin as its 125..250us pulse width expires. Timing ported verbatim from
// Beta 1.3 commandMotors().
void MotorsOneShot125::write(const State& s) {
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0, flagM2 = 0, flagM3 = 0, flagM4 = 0, flagM5 = 0, flagM6 = 0;

  digitalWrite(pins::M[0], HIGH);
  digitalWrite(pins::M[1], HIGH);
  digitalWrite(pins::M[2], HIGH);
  digitalWrite(pins::M[3], HIGH);
  digitalWrite(pins::M[4], HIGH);
  digitalWrite(pins::M[5], HIGH);
  pulseStart = micros();

  while (wentLow < 6) {
    timer = micros();
    if ((s.motor_pwm_us[0] <= timer - pulseStart) && (flagM1 == 0)) {
      digitalWrite(pins::M[0], LOW); wentLow++; flagM1 = 1;
    }
    if ((s.motor_pwm_us[1] <= timer - pulseStart) && (flagM2 == 0)) {
      digitalWrite(pins::M[1], LOW); wentLow++; flagM2 = 1;
    }
    if ((s.motor_pwm_us[2] <= timer - pulseStart) && (flagM3 == 0)) {
      digitalWrite(pins::M[2], LOW); wentLow++; flagM3 = 1;
    }
    if ((s.motor_pwm_us[3] <= timer - pulseStart) && (flagM4 == 0)) {
      digitalWrite(pins::M[3], LOW); wentLow++; flagM4 = 1;
    }
    if ((s.motor_pwm_us[4] <= timer - pulseStart) && (flagM5 == 0)) {
      digitalWrite(pins::M[4], LOW); wentLow++; flagM5 = 1;
    }
    if ((s.motor_pwm_us[5] <= timer - pulseStart) && (flagM6 == 0)) {
      digitalWrite(pins::M[5], LOW); wentLow++; flagM6 = 1;
    }
  }
}

void MotorsOneShot125::arm() {
  State s{};
  for (int i = 0; i < 6; i++) s.motor_pwm_us[i] = 125;
  for (int i = 0; i <= 50; i++) {
    write(s);
    delay(2);
  }
}

void MotorsOneShot125::calibrateEscs() {
  State s{};
  while (true) {
    for (int i = 0; i < 6; i++) s.motor_scaled[i] = 1.0f;
    scale(s);
    write(s);
    delayMicroseconds(500);
  }
}

void MotorsOneShot125::cut(State& s) {
  for (int i = 0; i < 6; i++) {
    s.motor_scaled[i] = 0.0f;
    s.motor_pwm_us[i] = 120;   // Beta 1.3: 120 (below 125) is hard cut
  }
}
