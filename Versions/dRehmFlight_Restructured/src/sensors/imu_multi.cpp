// Fuses N sub-IMUs (<=4) via median-of-two-closest per axis. Caveat: config.h
// holds one calibration set, applied by the base class to the fused output;
// run calibrate() with MultiImu active so constants represent post-fusion bias.

#include "imu_multi.h"

static constexpr uint8_t MAX_SUB = 4;

static void insertion_sort4(float* v, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    float x = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
}

static float fuse(float* v, uint8_t n) {
  if (n == 1) return v[0];
  if (n == 2) return 0.5f * (v[0] + v[1]);
  insertion_sort4(v, n);
  uint8_t mid = n / 2;
  float med = v[mid];
  float lo = v[mid - 1];
  float hi = (mid + 1 < n) ? v[mid + 1] : v[mid];
  float d_lo = med - lo;
  float d_hi = hi - med;
  float neighbor = (d_lo <= d_hi) ? lo : hi;
  return 0.5f * (med + neighbor);
}

MultiImu::MultiImu(IMU** sub_imus, uint8_t count) : sub_(sub_imus), count_(count) {}

bool MultiImu::begin() {
  if (count_ == 0 || count_ > MAX_SUB) return false;
  for (uint8_t i = 0; i < count_; i++) {
    if (!sub_[i]->begin()) return false;
  }
  return true;
}

bool MultiImu::hasMag() const {
  for (uint8_t i = 0; i < count_; i++) {
    if (sub_[i]->hasMag()) return true;
  }
  return false;
}

void MultiImu::readRaw(Vec3& gyro, Vec3& accel, Vec3& mag) {
  float gx[MAX_SUB], gy[MAX_SUB], gz[MAX_SUB];
  float ax[MAX_SUB], ay[MAX_SUB], az[MAX_SUB];
  float mx[MAX_SUB], my[MAX_SUB], mz[MAX_SUB];
  uint8_t n_mag = 0;

  for (uint8_t i = 0; i < count_; i++) {
    Vec3 g, a, m;
    sub_[i]->readRaw(g, a, m);
    gx[i] = g.x; gy[i] = g.y; gz[i] = g.z;
    ax[i] = a.x; ay[i] = a.y; az[i] = a.z;
    if (sub_[i]->hasMag()) {
      mx[n_mag] = m.x; my[n_mag] = m.y; mz[n_mag] = m.z;
      n_mag++;
    }
  }

  gyro.x  = fuse(gx, count_); gyro.y  = fuse(gy, count_); gyro.z  = fuse(gz, count_);
  accel.x = fuse(ax, count_); accel.y = fuse(ay, count_); accel.z = fuse(az, count_);

  if (n_mag > 0) {
    mag.x = fuse(mx, n_mag);
    mag.y = fuse(my, n_mag);
    mag.z = fuse(mz, n_mag);
  } else {
    mag.x = 0; mag.y = 0; mag.z = 0;
  }
}
