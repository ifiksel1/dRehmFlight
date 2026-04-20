#pragma once

#include "rc.h"
#include "../SBUS/SBUS.h"

class RcSbus : public Rc {
 public:
  bool begin() override;
  void readRaw(uint16_t channels[16], uint8_t& num_channels) override;

 private:
  SBUS sbus_{Serial5};
  uint16_t last_[16] = {0};
};
