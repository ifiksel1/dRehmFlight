#pragma once

#include "rc.h"

class RcPwm : public Rc {
 public:
  bool begin() override;
  void readRaw(uint16_t channels[16], uint8_t& num_channels) override;
};
