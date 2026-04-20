#include "blackbox.h"

#if defined(ARDUINO_TEENSY41) || defined(__IMXRT1062__)

#include "../../config.h"
#include <Arduino.h>
#include <SD.h>

namespace blackbox {

// Fixed-size binary frame. Packed so sizeof() matches on-disk layout exactly;
// decoder relies on little-endian IMXRT1062 byte order.
struct __attribute__((packed)) LogFrame {
  uint32_t t_us;
  float    gyro_x,  gyro_y,  gyro_z;
  float    accel_x, accel_y, accel_z;
  float    roll,    pitch,   yaw;
  float    roll_sp, pitch_sp, yaw_sp;
  float    roll_pid, pitch_pid, yaw_pid;
  uint16_t rc[8];
  float    motor_cmd[6];
  uint32_t motor_erpm[6];
  uint8_t  motor_telem_valid_mask;
  uint8_t  mode;
  uint8_t  armed;
  uint8_t  rc_failsafe;
};

// Ring: single-producer (task_control via log_frame) / single-consumer (flush).
// head_/tail_ are not atomic — safe because each side touches only its own index
// and reads the other as a plain uint32_t snapshot. Never block the control loop.
static constexpr uint32_t RING_SZ = 4096;
static constexpr uint32_t WRITE_CHUNK = 512;   // SD native block size

static File              log_file_;
static uint8_t           ring_[RING_SZ];
static volatile uint32_t head_ = 0;
static volatile uint32_t tail_ = 0;
static uint32_t          dropped_frames_ = 0;
static bool              logging_ = false;

static uint16_t next_log_index() {
  uint16_t max_idx = 0;
  File root = SD.open("/");
  if (!root) return 1;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    const char* n = f.name();
    if (n && (n[0] == 'L' || n[0] == 'l') &&
        (n[1] == 'O' || n[1] == 'o') &&
        (n[2] == 'G' || n[2] == 'g')) {
      uint16_t v = 0;
      for (int i = 3; i < 7 && n[i] >= '0' && n[i] <= '9'; i++)
        v = v * 10 + (n[i] - '0');
      if (v > max_idx) max_idx = v;
    }
    f.close();
  }
  root.close();
  return max_idx + 1;
}

static void write_header() {
  char line[192];

  log_file_.print(F("# dRehmFlight blackbox v1\n"));

  snprintf(line, sizeof(line), "# control_rate_hz=%lu\n",
           (unsigned long)CONTROL_RATE_HZ);
  log_file_.print(line);

  snprintf(line, sizeof(line), "# start_time=%lu\n", (unsigned long)millis());
  log_file_.print(line);

  snprintf(line, sizeof(line), "# frame_size=%u\n",
           (unsigned)sizeof(LogFrame));
  log_file_.print(line);

  log_file_.print(F(
    "# fields: t_us,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,"
    "roll,pitch,yaw,roll_sp,pitch_sp,yaw_sp,roll_pid,pitch_pid,yaw_pid,"
    "rc0,rc1,rc2,rc3,rc4,rc5,rc6,rc7,"
    "motor_cmd0,motor_cmd1,motor_cmd2,motor_cmd3,motor_cmd4,motor_cmd5,"
    "motor_erpm0,motor_erpm1,motor_erpm2,motor_erpm3,motor_erpm4,motor_erpm5,"
    "motor_telem_valid_mask,mode,armed,rc_failsafe\n"));

  log_file_.print(F(
    "# types: u32,f,f,f,f,f,f,f,f,f,f,f,f,f,f,f,"
    "u16,u16,u16,u16,u16,u16,u16,u16,"
    "f,f,f,f,f,f,u32,u32,u32,u32,u32,u32,u8,u8,u8,u8\n"));

  log_file_.print(F("# ----BEGIN BINARY----\n"));
}

bool begin() {
  if (logging_) return true;
  if (!SD.begin(BUILTIN_SDCARD)) return false;

  char name[16];
  snprintf(name, sizeof(name), "LOG%04u.BBL", next_log_index());
  log_file_ = SD.open(name, FILE_WRITE);
  if (!log_file_) return false;

  head_ = tail_ = 0;
  dropped_frames_ = 0;
  write_header();
  log_file_.flush();
  logging_ = true;
  return true;
}

void log_frame(State& s) {
  if (!logging_ || !s.armed) return;

  LogFrame f;
  f.t_us     = s.t_us;
  f.gyro_x   = s.gyro.x;  f.gyro_y   = s.gyro.y;  f.gyro_z   = s.gyro.z;
  f.accel_x  = s.accel.x; f.accel_y  = s.accel.y; f.accel_z  = s.accel.z;
  f.roll     = s.euler.x; f.pitch    = s.euler.y; f.yaw      = s.euler.z;
  f.roll_sp  = s.roll_des;  f.pitch_sp = s.pitch_des; f.yaw_sp = s.yaw_des;
  f.roll_pid = s.roll_pid;  f.pitch_pid = s.pitch_pid; f.yaw_pid = s.yaw_pid;
  for (int i = 0; i < 8; i++) f.rc[i] = s.rc[i];
  uint8_t mask = 0;
  for (int i = 0; i < 6; i++) {
    f.motor_cmd[i]  = s.motor_scaled[i];
    f.motor_erpm[i] = s.motor_erpm[i];
    if (s.motor_telem_valid[i]) mask |= (1u << i);
  }
  f.motor_telem_valid_mask = mask;
  f.mode        = (uint8_t)s.mode;
  f.armed       = s.armed ? 1 : 0;
  f.rc_failsafe = s.rc_failsafe ? 1 : 0;

  const uint32_t head  = head_;
  const uint32_t tail  = tail_;
  const uint32_t used  = head - tail;
  if (used + sizeof(LogFrame) > RING_SZ) { dropped_frames_++; return; }

  const uint32_t off   = head % RING_SZ;
  const uint32_t first = (off + sizeof(LogFrame) <= RING_SZ)
                       ? sizeof(LogFrame) : (RING_SZ - off);
  memcpy(&ring_[off], &f, first);
  if (first < sizeof(LogFrame))
    memcpy(&ring_[0], (uint8_t*)&f + first, sizeof(LogFrame) - first);
  head_ = head + sizeof(LogFrame);
}

void flush() {
  if (!logging_) return;

  const uint32_t head = head_;
  uint32_t tail = tail_;
  uint32_t avail = head - tail;
  if (avail < WRITE_CHUNK) return;   // wait for a full block — amortize FAT cost

  while (avail >= WRITE_CHUNK) {
    const uint32_t off   = tail % RING_SZ;
    const uint32_t first = (off + WRITE_CHUNK <= RING_SZ)
                         ? WRITE_CHUNK : (RING_SZ - off);
    log_file_.write(&ring_[off], first);
    if (first < WRITE_CHUNK)
      log_file_.write(&ring_[0], WRITE_CHUNK - first);
    tail  += WRITE_CHUNK;
    avail -= WRITE_CHUNK;
  }
  tail_ = tail;
  log_file_.flush();
}

void stop() {
  if (!logging_) return;

  // Drain tail -> head, even partial block, then close.
  const uint32_t head = head_;
  uint32_t tail = tail_;
  while (tail < head) {
    const uint32_t remain = head - tail;
    const uint32_t off    = tail % RING_SZ;
    const uint32_t chunk  = (remain < RING_SZ - off) ? remain : (RING_SZ - off);
    log_file_.write(&ring_[off], chunk);
    tail += chunk;
  }
  tail_ = tail;
  log_file_.flush();
  log_file_.close();
  logging_ = false;
}

bool is_logging() { return logging_; }

}  // namespace blackbox

#else
#pragma message("Blackbox disabled: no SD on this target")

namespace blackbox {
  bool begin() { return false; }
  void log_frame(State&) {}
  void flush() {}
  void stop() {}
  bool is_logging() { return false; }
}

#endif
