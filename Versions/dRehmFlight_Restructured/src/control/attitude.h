#pragma once

#include "../../state.h"

namespace attitude {
  void madgwick(State& s);
  void madgwick6dof(State& s);
}
