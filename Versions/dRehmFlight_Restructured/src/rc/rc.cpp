#include "rc.h"
#include "../../config.h"

void Rc::update(State& s) {
  uint16_t ch[16] = {0};
  uint8_t n = 0;
  readRaw(ch, n);

  for (uint8_t i = 0; i < n && i < 16; ++i) s.rc[i] = ch[i];

  const uint16_t fs[6] = {CH1_FS, CH2_FS, CH3_FS, CH4_FS, CH5_FS, CH6_FS};
  bool fail = false;
  for (uint8_t i = 0; i < 6; ++i) {
    if (s.rc[i] < 800 || s.rc[i] > 2200) { fail = true; break; }
  }
  if (fail) {
    for (uint8_t i = 0; i < 6; ++i) s.rc[i] = fs[i];
    s.rc_failsafe = true;
  } else {
    s.rc_failsafe = false;
  }
}
