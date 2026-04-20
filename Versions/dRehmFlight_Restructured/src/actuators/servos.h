#pragma once

#include "../../state.h"
#include <PWMServo.h>

class Servos {
 public:
  void begin();
  void scale(State& s);
  void write(const State& s);

 private:
  PWMServo servo_[7];
};
