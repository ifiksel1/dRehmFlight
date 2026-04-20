#include "rc_ppm.h"
#include "../../config.h"
#include <Arduino.h>

static volatile unsigned long ppm_buf[6];
static volatile int ppm_counter = 0;
static volatile unsigned long ppm_last_us = 0;

static void getPPM() {
  if (digitalRead(pins::PPM) != 1) return;
  unsigned long now = micros();
  unsigned long dt = now - ppm_last_us;
  ppm_last_us = now;

  if (dt > 5000) ppm_counter = 0;

  if (ppm_counter >= 1 && ppm_counter <= 6) {
    ppm_buf[ppm_counter - 1] = dt;
  }
  ppm_counter++;
}

bool RcPpm::begin() {
  pinMode(pins::PPM, INPUT_PULLUP);
  delay(20);
  attachInterrupt(digitalPinToInterrupt(pins::PPM), getPPM, CHANGE);
  return true;
}

void RcPpm::readRaw(uint16_t channels[16], uint8_t& num_channels) {
  noInterrupts();
  for (uint8_t i = 0; i < 6; ++i) channels[i] = (uint16_t)ppm_buf[i];
  interrupts();
  num_channels = 6;
}
