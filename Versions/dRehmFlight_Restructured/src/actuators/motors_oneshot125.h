#pragma once

#include "motors.h"

class MotorsOneShot125 : public Motors {
 public:
  bool begin() override;
  void scale(State& s) override;
  void write(const State& s) override;
  void arm() override;
  void cut(State& s) override;
  void calibrateEscs() override;
};
