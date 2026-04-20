#include "rc_sbus.h"

bool RcSbus::begin() {
  sbus_.begin();
  return true;
}

void RcSbus::readRaw(uint16_t channels[16], uint8_t& num_channels) {
  uint16_t raw[16];
  bool failsafe = false;
  bool lost_frame = false;

  if (sbus_.read(&raw[0], &failsafe, &lost_frame)) {
    const float scale = 0.615f;
    const float bias  = 895.0f;
    for (uint8_t i = 0; i < 16; ++i) {
      last_[i] = (uint16_t)(raw[i] * scale + bias);
    }
  }
  for (uint8_t i = 0; i < 16; ++i) channels[i] = last_[i];
  num_channels = 16;
}
