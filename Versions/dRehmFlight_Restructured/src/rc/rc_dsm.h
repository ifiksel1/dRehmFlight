#pragma once

#include "rc.h"
#include "../DSMRX/DSMRX.h"

class RcDsm : public Rc {
 public:
  bool begin() override;
  void readRaw(uint16_t channels[16], uint8_t& num_channels) override;

 private:
  uint16_t last_[16] = {0};
};
