#pragma once
#include <Arduino.h>
#include "../state.h"

using TaskFn = void (*)(State&);

struct Task {
  uint32_t period_us;       // 1e6 / rate_hz
  uint32_t next_due_us;     // next scheduled call
  TaskFn   fn;
  const char* name;         // for debugging; may be null
};

class Scheduler {
 public:
  // tasks[]: caller owns the array and its lifetime. The first task is treated
  // as the "control" task and enforces deterministic loop rate via busy-wait.
  void begin(Task* tasks, uint8_t count, uint32_t control_rate_hz);

  // Call from loop(). Executes the control task at its fixed rate and
  // dispatches any lower-rate tasks whose next_due_us has elapsed.
  void tick(State& s);

 private:
  Task*   tasks_ = nullptr;
  uint8_t count_ = 0;
  uint32_t control_period_us_ = 0;
  uint32_t next_control_due_us_ = 0;
};
