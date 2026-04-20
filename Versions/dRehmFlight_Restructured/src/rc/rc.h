#pragma once

#include <stdint.h>
#include "../../state.h"

class Rc {
 public:
  virtual ~Rc() = default;

  virtual bool begin() = 0;
  virtual void readRaw(uint16_t channels[16], uint8_t& num_channels) = 0;

  void update(State& s);
};
