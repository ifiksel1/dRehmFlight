#pragma once
#include <stdint.h>

namespace config_store {

struct Data {
  uint32_t magic;
  uint16_t version;
  uint16_t size;

  float acc_err[3];
  float gyro_err[3];
  float mag_err[3];
  float mag_scale[3];

  float kp_roll_angle,  ki_roll_angle,  kd_roll_angle;
  float kp_pitch_angle, ki_pitch_angle, kd_pitch_angle;
  float kp_roll_rate,   ki_roll_rate,   kd_roll_rate;
  float kp_pitch_rate,  ki_pitch_rate,  kd_pitch_rate;
  float kp_yaw,         ki_yaw,         kd_yaw;

  // CRC covers all fields above; keep last so save() can stamp it in place.
  uint32_t crc32;
};

extern Data data;

bool begin();
bool save();
void load_defaults();

}  // namespace config_store
