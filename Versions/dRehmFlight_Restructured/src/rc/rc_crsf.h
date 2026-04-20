#pragma once

#include "rc.h"

class RcCrsf : public Rc {
 public:
  bool begin() override;
  void readRaw(uint16_t channels[16], uint8_t& num_channels) override;
  void send_telemetry(const State& s);

 private:
  void poll();
  bool parse_frame();
  void send_frame(uint8_t type, const uint8_t* payload, uint8_t payload_len);

  uint8_t  buf_[64];
  uint8_t  buf_len_ = 0;
  uint16_t last_[16] = {0};
  uint32_t last_frame_ms_ = 0;
};
