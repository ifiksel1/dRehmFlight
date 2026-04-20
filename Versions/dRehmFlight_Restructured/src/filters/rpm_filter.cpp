#include "rpm_filter.h"
#include "../../config.h"
#include <Arduino.h>
#include <math.h>

static constexpr int NUM_MOTORS    = 6;
static constexpr int NUM_AXES      = 3;
static constexpr int NUM_HARMONICS = 3;
static constexpr float Q_FACTOR    = 500.0f;
static constexpr float F_MIN_HZ    = 50.0f;
static constexpr float F_MAX_HZ    = (float)CONTROL_RATE_HZ / 2.2f;
static constexpr float F_UPDATE_THRESHOLD_HZ = 2.0f;  // recompute coeffs when center shifts > 2 Hz

struct Biquad {
  float b0, b1, b2, a1, a2;
  float z1, z2;
};

// [motor][harmonic][axis]. Last cached center freq is per motor+harmonic
// (axis-independent — all three axes share the same coefficients).
static Biquad notches_[NUM_MOTORS][NUM_HARMONICS][NUM_AXES];
static float  last_f_hz_[NUM_MOTORS][NUM_HARMONICS];
static bool   notch_active_[NUM_MOTORS];

static inline void compute_coeffs(Biquad& bx, Biquad& by, Biquad& bz, float f_hz) {
  // RBJ audio EQ cookbook, notch form.
  const float w0    = 2.0f * (float)M_PI * f_hz / (float)CONTROL_RATE_HZ;
  const float cw    = cosf(w0);
  const float sw    = sinf(w0);
  const float alpha = sw / (2.0f * Q_FACTOR);
  const float a0    = 1.0f + alpha;
  const float inv   = 1.0f / a0;
  const float b0    =  1.0f * inv;
  const float b1    = -2.0f * cw * inv;
  const float b2    =  1.0f * inv;
  const float a1    = -2.0f * cw * inv;
  const float a2    = (1.0f - alpha) * inv;
  bx.b0 = b0; bx.b1 = b1; bx.b2 = b2; bx.a1 = a1; bx.a2 = a2;
  by.b0 = b0; by.b1 = b1; by.b2 = b2; by.a1 = a1; by.a2 = a2;
  bz.b0 = b0; bz.b1 = b1; bz.b2 = b2; bz.a1 = a1; bz.a2 = a2;
}

static inline float biquad_step(Biquad& b, float x) {
  // Transposed direct form II.
  const float y = b.b0 * x + b.z1;
  b.z1 = b.b1 * x - b.a1 * y + b.z2;
  b.z2 = b.b2 * x - b.a2 * y;
  return y;
}

void rpm_filter::init() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    notch_active_[i] = false;
    for (int h = 0; h < NUM_HARMONICS; h++) {
      last_f_hz_[i][h] = 0.0f;
      for (int a = 0; a < NUM_AXES; a++) {
        notches_[i][h][a] = Biquad{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      }
    }
  }
}

void rpm_filter::apply(State& s) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    // Bypass: no valid telemetry this tick, or fundamental outside usable band.
    const float f_fund = ((float)s.motor_erpm[i] / (float)MOTOR_POLE_PAIRS) / 60.0f;
    if (!s.motor_telem_valid[i] || f_fund < F_MIN_HZ) {
      notch_active_[i] = false;
      continue;
    }
    notch_active_[i] = true;

    for (int h = 0; h < NUM_HARMONICS; h++) {
      const float f_hz = f_fund * (float)(h + 1);
      if (f_hz > F_MAX_HZ) {
        // Harmonic above Nyquist margin — zero its taps so the cascade is a pass-through.
        for (int a = 0; a < NUM_AXES; a++) {
          notches_[i][h][a] = Biquad{1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                     notches_[i][h][a].z1, notches_[i][h][a].z2};
        }
        last_f_hz_[i][h] = 0.0f;
        continue;
      }
      if (fabsf(f_hz - last_f_hz_[i][h]) > F_UPDATE_THRESHOLD_HZ) {
        compute_coeffs(notches_[i][h][0], notches_[i][h][1], notches_[i][h][2], f_hz);
        last_f_hz_[i][h] = f_hz;
      }
    }
  }

  float gx = s.gyro.x, gy = s.gyro.y, gz = s.gyro.z;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!notch_active_[i]) continue;
    for (int h = 0; h < NUM_HARMONICS; h++) {
      gx = biquad_step(notches_[i][h][0], gx);
      gy = biquad_step(notches_[i][h][1], gy);
      gz = biquad_step(notches_[i][h][2], gz);
    }
  }
  s.gyro.x = gx;
  s.gyro.y = gy;
  s.gyro.z = gz;
}
