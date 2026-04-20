#include "config_store.h"
#include "../../config.h"
#include <EEPROM.h>
#include <stddef.h>
#include <string.h>

namespace config_store {

static constexpr uint32_t MAGIC   = 0x4D485244;  // 'DRHM' LE
static constexpr uint16_t VERSION = 1;

Data data;

static uint32_t crc_table_[256];
static bool     crc_ready_ = false;

static void crc_init() {
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int j = 0; j < 8; ++j) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    crc_table_[i] = c;
  }
  crc_ready_ = true;
}

static uint32_t crc32(const uint8_t* p, size_t n) {
  if (!crc_ready_) crc_init();
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) c = crc_table_[(c ^ p[i]) & 0xFF] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

// CRC covers bytes [0 .. offsetof(Data, crc32)).
static inline uint32_t compute_crc(const Data& d) {
  return crc32(reinterpret_cast<const uint8_t*>(&d), offsetof(Data, crc32));
}

void load_defaults() {
  data.magic   = MAGIC;
  data.version = VERSION;
  data.size    = sizeof(Data);

  data.acc_err[0]   = ACC_ERR_X;
  data.acc_err[1]   = ACC_ERR_Y;
  data.acc_err[2]   = ACC_ERR_Z;
  data.gyro_err[0]  = GYRO_ERR_X;
  data.gyro_err[1]  = GYRO_ERR_Y;
  data.gyro_err[2]  = GYRO_ERR_Z;
  data.mag_err[0]   = MAG_ERR_X;
  data.mag_err[1]   = MAG_ERR_Y;
  data.mag_err[2]   = MAG_ERR_Z;
  data.mag_scale[0] = MAG_SCALE_X;
  data.mag_scale[1] = MAG_SCALE_Y;
  data.mag_scale[2] = MAG_SCALE_Z;

  data.kp_roll_angle  = KP_ROLL_ANGLE;
  data.ki_roll_angle  = KI_ROLL_ANGLE;
  data.kd_roll_angle  = KD_ROLL_ANGLE;
  data.kp_pitch_angle = KP_PITCH_ANGLE;
  data.ki_pitch_angle = KI_PITCH_ANGLE;
  data.kd_pitch_angle = KD_PITCH_ANGLE;
  data.kp_roll_rate   = KP_ROLL_RATE;
  data.ki_roll_rate   = KI_ROLL_RATE;
  data.kd_roll_rate   = KD_ROLL_RATE;
  data.kp_pitch_rate  = KP_PITCH_RATE;
  data.ki_pitch_rate  = KI_PITCH_RATE;
  data.kd_pitch_rate  = KD_PITCH_RATE;
  data.kp_yaw         = KP_YAW;
  data.ki_yaw         = KI_YAW;
  data.kd_yaw         = KD_YAW;

  data.crc32 = compute_crc(data);
}

bool begin() {
  Data scratch;
  for (size_t i = 0; i < sizeof(Data); ++i) {
    reinterpret_cast<uint8_t*>(&scratch)[i] = EEPROM.read(i);
  }
  if (scratch.magic == MAGIC &&
      scratch.version == VERSION &&
      scratch.size == sizeof(Data) &&
      compute_crc(scratch) == scratch.crc32) {
    data = scratch;
    return true;
  }
  load_defaults();
  save();
  return false;
}

bool save() {
  data.crc32 = compute_crc(data);
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&data);
  for (size_t i = 0; i < sizeof(Data); ++i) EEPROM.update(i, p[i]);
  return true;
}

}  // namespace config_store
