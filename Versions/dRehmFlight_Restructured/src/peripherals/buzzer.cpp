#include "buzzer.h"
#include "../../config.h"
#include <Arduino.h>

namespace buzzer {

// Each Step is {freq_hz, duration_ms}. freq_hz=0 means silence.
struct Step { uint16_t freq_hz; uint16_t duration_ms; };

static const Step ARMED_SEQ[]      = {{2000, 100}, {0, 0}};
static const Step DISARMED_SEQ[]   = {{2000, 100}, {0, 80}, {2000, 100}, {0, 0}};
static const Step REFUSED_SEQ[]    = {{400, 500}, {0, 0}};
static const Step LOW_SEQ[]        = {{1500, 150}, {0, 850}};          // loops
static const Step CRITICAL_SEQ[]   = {{2500, 60}, {0, 60}};            // loops
static const Step FAILSAFE_SEQ[]   = {{500, 200}, {2000, 200}};        // loops
static const Step BEACON_SEQ[]     = {{4000, 500}, {0, 1500}};         // loops

// Patterns whose final step has duration_ms != 0 loop; duration_ms == 0
// terminates (one-shot).
static const Step* const SEQS[] = {
  nullptr, ARMED_SEQ, DISARMED_SEQ, REFUSED_SEQ,
  LOW_SEQ, CRITICAL_SEQ, FAILSAFE_SEQ, BEACON_SEQ
};
static const uint8_t SEQ_LEN[] = {
  0, 2, 4, 2, 2, 2, 2, 2
};

static Pattern current_ = Pattern::NONE;
static uint8_t  step_   = 0;
static uint32_t step_end_ms_ = 0;
static bool     tone_on_     = false;

static void stop_tone() {
  if (tone_on_) {
#if defined(BOARD_TEENSY41_DEFAULT)
    noTone(pins::BUZZER);
#else
    if (pins::BUZZER >= 0) noTone(pins::BUZZER);
#endif
    tone_on_ = false;
  }
}

void begin() {
#if !defined(BOARD_TEENSY41_DEFAULT)
  if (pins::BUZZER < 0) return;
#endif
  pinMode(pins::BUZZER, OUTPUT);
}

void play(Pattern p) {
  if (p == current_) return;
  current_ = p;
  step_    = 0;
  step_end_ms_ = 0;   // forces update() to start step 0 immediately
  stop_tone();
}

void update() {
  if (current_ == Pattern::NONE) { stop_tone(); return; }

  const uint8_t idx = (uint8_t)current_;
  const Step* seq   = SEQS[idx];
  const uint8_t len = SEQ_LEN[idx];
  const uint32_t now = millis();

  if ((int32_t)(now - step_end_ms_) < 0) return;

  if (step_ >= len) {
    // End of pattern. Loop if last step has nonzero duration, else stop.
    if (seq[len - 1].duration_ms > 0) step_ = 0;
    else { current_ = Pattern::NONE; stop_tone(); return; }
  }

  const Step& s = seq[step_];
  stop_tone();
  if (s.freq_hz > 0) {
#if !defined(BOARD_TEENSY41_DEFAULT)
    if (pins::BUZZER >= 0)
#endif
    { tone(pins::BUZZER, s.freq_hz); tone_on_ = true; }
  }
  step_end_ms_ = now + s.duration_ms;
  step_++;
}

}  // namespace buzzer
