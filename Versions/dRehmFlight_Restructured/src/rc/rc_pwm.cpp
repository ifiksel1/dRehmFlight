#include "rc_pwm.h"
#include "../../config.h"
#include <Arduino.h>

static volatile unsigned long rising_1, rising_2, rising_3, rising_4, rising_5, rising_6;
static volatile unsigned long ch1_raw, ch2_raw, ch3_raw, ch4_raw, ch5_raw, ch6_raw;

static void getCh1() {
  if (digitalRead(pins::CH1) == 1) rising_1 = micros();
  else ch1_raw = micros() - rising_1;
}
static void getCh2() {
  if (digitalRead(pins::CH2) == 1) rising_2 = micros();
  else ch2_raw = micros() - rising_2;
}
static void getCh3() {
  if (digitalRead(pins::CH3) == 1) rising_3 = micros();
  else ch3_raw = micros() - rising_3;
}
static void getCh4() {
  if (digitalRead(pins::CH4) == 1) rising_4 = micros();
  else ch4_raw = micros() - rising_4;
}
static void getCh5() {
  if (digitalRead(pins::CH5) == 1) rising_5 = micros();
  else ch5_raw = micros() - rising_5;
}
static void getCh6() {
  if (digitalRead(pins::CH6) == 1) rising_6 = micros();
  else ch6_raw = micros() - rising_6;
}

bool RcPwm::begin() {
  pinMode(pins::CH1, INPUT_PULLUP);
  pinMode(pins::CH2, INPUT_PULLUP);
  pinMode(pins::CH3, INPUT_PULLUP);
  pinMode(pins::CH4, INPUT_PULLUP);
  pinMode(pins::CH5, INPUT_PULLUP);
  pinMode(pins::CH6, INPUT_PULLUP);
  delay(20);
  attachInterrupt(digitalPinToInterrupt(pins::CH1), getCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::CH2), getCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::CH3), getCh3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::CH4), getCh4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::CH5), getCh5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::CH6), getCh6, CHANGE);
  delay(20);
  return true;
}

void RcPwm::readRaw(uint16_t channels[16], uint8_t& num_channels) {
  noInterrupts();
  channels[0] = (uint16_t)ch1_raw;
  channels[1] = (uint16_t)ch2_raw;
  channels[2] = (uint16_t)ch3_raw;
  channels[3] = (uint16_t)ch4_raw;
  channels[4] = (uint16_t)ch5_raw;
  channels[5] = (uint16_t)ch6_raw;
  interrupts();
  num_channels = 6;
}
