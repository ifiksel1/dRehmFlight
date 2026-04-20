#pragma once

#include "../../state.h"

namespace blackbox {
  bool begin();              // init SD, open next numbered log, write header
  void log_frame(State& s);  // called from task_control at end of tick
  void flush();              // periodic flush so power loss loses <1s of data
  void stop();               // close file (arm->disarm transition)
  bool is_logging();
}
