#include "scheduler.h"

static uint32_t prev_control_us = 0;

void Scheduler::begin(Task* tasks, uint8_t count, uint32_t control_rate_hz) {
  tasks_ = tasks;
  count_ = count;
  control_period_us_ = 1000000UL / control_rate_hz;

  uint32_t now = micros();
  next_control_due_us_ = now;
  prev_control_us = now;
  for (uint8_t i = 0; i < count_; i++) {
    tasks_[i].next_due_us = now;
  }
}

void Scheduler::tick(State& s) {
  // Busy-wait until the control task is due (wrap-safe via signed diff).
  while ((int32_t)(micros() - next_control_due_us_) < 0) {}

  uint32_t now = micros();
  s.dt = (now - prev_control_us) / 1e6f;
  s.t_us = now;
  prev_control_us = now;

  next_control_due_us_ += control_period_us_;
  // Underrun resync: if we're already past the next deadline, drop the
  // backlog and reschedule one period out to avoid spiral-of-death.
  if ((int32_t)(now - next_control_due_us_) >= 0) {
    next_control_due_us_ = now + control_period_us_;
  }

  if (count_ == 0) return;
  tasks_[0].fn(s);

  for (uint8_t i = 1; i < count_; i++) {
    Task& t = tasks_[i];
    // Wrap-safe "is it due yet?" comparison.
    if ((int32_t)(now - t.next_due_us) >= 0) {
      t.fn(s);
      t.next_due_us += t.period_us;
      // Underrun resync for lower-rate tasks.
      if ((int32_t)(now - t.next_due_us) >= 0) {
        t.next_due_us = now + t.period_us;
      }
    }
  }
}
