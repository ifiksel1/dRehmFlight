#include "rc_dsm.h"
#include "../../config.h"
#include <Arduino.h>

static DSM1024 dsm;

void serialEvent3() {
  while (Serial3.available()) {
    dsm.handleSerialEvent(Serial3.read(), micros());
  }
}

bool RcDsm::begin() {
  Serial3.begin(115000);
  return true;
}

void RcDsm::readRaw(uint16_t channels[16], uint8_t& num_channels) {
  if (!dsm.timedOut(micros()) && dsm.gotNewFrame()) {
    uint16_t values[DSM_NUM_CHANNELS];
    dsm.getChannelValues(values, DSM_NUM_CHANNELS);
    for (uint8_t i = 0; i < DSM_NUM_CHANNELS && i < 16; ++i) {
      last_[i] = values[i];
    }
  }
  for (uint8_t i = 0; i < 16; ++i) channels[i] = last_[i];
  num_channels = DSM_NUM_CHANNELS;
}
