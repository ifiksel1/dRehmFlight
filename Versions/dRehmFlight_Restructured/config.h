#pragma once

#include <stdint.h>
#include <stddef.h>

// =============================================================================
// USER CONFIGURATION — edit this file for your build
// =============================================================================

// ---- Sensor selection (pick exactly one per category) -----------------------

#define IMU_MPU6050
// #define IMU_MPU9250
// #define IMU_BMI270
// #define IMU_ICM20948      // has onboard mag; MAG_* is ignored
// #define IMU_ICM42688
// #define IMU_MULTI         // fuse multiple IMUs — edit sub-IMU list in the .ino

// #define MAG_NONE
#define MAG_NONE
// #define MAG_QMC5883L

// #define BARO_NONE
#define BARO_NONE
// #define BARO_BMP585

// ---- Motor protocol (pick one) ----------------------------------------------

#define MOTORS_ONESHOT125
// #define MOTORS_DSHOT

// DShot bitrate (kbps). Only used if MOTORS_DSHOT is selected.
static constexpr uint32_t DSHOT_KBPS = 600;   // 150, 300, 600

// Motor pole-pair count, for converting DShot eRPM -> mechanical RPM.
static constexpr uint8_t MOTOR_POLE_PAIRS = 7;   // 14-pole brushless = 7 pole pairs

#define RC_PWM
// #define RC_PPM
// #define RC_SBUS
// #define RC_DSM
// #define RC_CRSF   // Crossfire / ExpressLRS — bidirectional; enables telemetry
static constexpr uint8_t DSM_NUM_CHANNELS = 6;

// ---- Full-scale ranges ------------------------------------------------------

#define GYRO_FS_DPS   250     // 250, 500, 1000, 2000
#define ACCEL_FS_G    2       // 2, 4, 8, 16

// ---- Loop rates (Hz) --------------------------------------------------------
//
// CONTROL_RATE_HZ sets the period of the primary loop and must accommodate
// the worst-case task_control runtime, including motor write + inline DShot
// telemetry readback. Budget at DShot600 with 6 motors sequential:
//   per motor:  16-bit frame  ~27us + 21-bit GCR response ~58us = ~85us
//   6 motors:   ~510us
// At 2000 Hz the period is only 500us — tight-to-over-budget. At 1500 Hz
// (667us period) we have ~150us headroom for RPM filter + Madgwick + PID.
// Drop further to 1000 Hz if you add heavy sensors (GPS parsing, EKF, etc.)

static constexpr uint32_t CONTROL_RATE_HZ = 1500;
static constexpr uint32_t BARO_RATE_HZ    = 100;
static constexpr uint32_t MAG_RATE_HZ     = 100;
static constexpr uint32_t TELEM_RATE_HZ   = 50;

// ---- Filter betas (first-order LP on raw sensor reads) ----------------------

static constexpr float B_MADGWICK = 0.04f;
static constexpr float B_ACCEL    = 0.14f;   // MPU6050: 0.14, MPU9250: 0.20
static constexpr float B_GYRO     = 0.10f;   // MPU6050: 0.10, MPU9250: 0.17
static constexpr float B_MAG      = 1.00f;

// ---- IMU calibration offsets (run calibrate_imu() once, paste values) -------

static constexpr float ACC_ERR_X  = 0.0f;
static constexpr float ACC_ERR_Y  = 0.0f;
static constexpr float ACC_ERR_Z  = 0.0f;
static constexpr float GYRO_ERR_X = 0.0f;
static constexpr float GYRO_ERR_Y = 0.0f;
static constexpr float GYRO_ERR_Z = 0.0f;

// ---- Magnetometer calibration (hard/soft iron) ------------------------------

static constexpr float MAG_ERR_X   = 0.0f;
static constexpr float MAG_ERR_Y   = 0.0f;
static constexpr float MAG_ERR_Z   = 0.0f;
static constexpr float MAG_SCALE_X = 1.0f;
static constexpr float MAG_SCALE_Y = 1.0f;
static constexpr float MAG_SCALE_Z = 1.0f;

// ---- Controller limits and gains --------------------------------------------

static constexpr float I_LIMIT  = 25.0f;
static constexpr float MAX_ROLL  = 30.0f;    // stick-setpoint max: deg in ANGLE, deg/s in ACRO
static constexpr float MAX_PITCH = 30.0f;
static constexpr float MAX_YAW   = 160.0f;

// Betaflight-style hard ceiling enforced inside control_angle() —
// caps commanded tilt regardless of where the setpoint came from
// (stick, HORIZON blend, future auto modes). Should be >= MAX_ROLL/PITCH.
// Filter behavior degrades past ~70 deg; keep below that.
static constexpr float ANGLE_LIMIT_DEG = 55.0f;

static constexpr float KP_ROLL_ANGLE  = 0.20f;
static constexpr float KI_ROLL_ANGLE  = 0.30f;
static constexpr float KD_ROLL_ANGLE  = 0.05f;
static constexpr float KP_PITCH_ANGLE = 0.20f;
static constexpr float KI_PITCH_ANGLE = 0.30f;
static constexpr float KD_PITCH_ANGLE = 0.05f;

static constexpr float KP_ROLL_RATE   = 0.15f;
static constexpr float KI_ROLL_RATE   = 0.20f;
static constexpr float KD_ROLL_RATE   = 0.0002f;
static constexpr float KP_PITCH_RATE  = 0.15f;
static constexpr float KI_PITCH_RATE  = 0.20f;
static constexpr float KD_PITCH_RATE  = 0.0002f;

static constexpr float KP_YAW = 0.30f;
static constexpr float KI_YAW = 0.05f;
static constexpr float KD_YAW = 0.00015f;

// ---- Failsafe channel values (µs) -------------------------------------------

static constexpr uint16_t CH1_FS = 1000;  // throttle
static constexpr uint16_t CH2_FS = 1500;  // aileron
static constexpr uint16_t CH3_FS = 1500;  // elevator
static constexpr uint16_t CH4_FS = 1500;  // rudder
static constexpr uint16_t CH5_FS = 2000;  // gear — >1500 = throttle cut
static constexpr uint16_t CH6_FS = 2000;  // aux1

// ---- Pin assignments --------------------------------------------------------
// Default mapping is for the Spin 1 custom RT1062 flight controller (see
// pin_allocation.xlsx). To build for a bare Teensy 4.1 with the original
// Beta 1.3 layout instead, define BOARD_TEENSY41_DEFAULT before including this.

#if defined(BOARD_TEENSY41_DEFAULT)
namespace pins {
  constexpr int CH1 = 15, CH2 = 16, CH3 = 17, CH4 = 20, CH5 = 21, CH6 = 22;
  constexpr int PPM = 23;
  constexpr int M[6] = {0, 1, 2, 3, 4, 5};
  constexpr int SERVO[7] = {6, 7, 8, 9, 10, 11, 12};
  constexpr int IMU_CS = 36;
  constexpr int BUZZER = 8;              // user-wired; any GPIO
  constexpr int LED    = LED_BUILTIN;    // onboard LED on pin 13
  // CRSF / SBUS UART fallback
  #define CRSF_SERIAL Serial5
}
#else  // BOARD_CUSTOM_RT1062 (default)
namespace pins {
  // RC (only relevant when RC_PWM / RC_PPM selected — custom board uses CRSF)
  constexpr int CH1 = 15, CH2 = 16, CH3 = 17, CH4 = 20, CH5 = 21, CH6 = 22;
  constexpr int PPM = 23;

  // Motors — Spin 1 board: M1..M6 on pins 2..6,9; M7/M8 remapped to servos.
  constexpr int M[6] = {2, 3, 4, 5, 6, 9};

  // Servos — only 2 physical PWM outputs on this board (the former M7/M8).
  // State::servo_scaled[7] oversizes this; only SERVO_COUNT entries are
  // actually driven on hardware.
  constexpr int SERVO[2] = {36, 37};

  // ICM-42688 primary IMU on LPSPI4 (Teensy's default SPI bus, pins 11/12/13).
  constexpr int IMU_CS = 10;

  // I²C secondary IMUs / baro / mag interrupts
  constexpr int IMU_INT_PRIMARY  = 33;   // ICM-42688 data-ready
  constexpr int IMU_INT_BMI270   = 27;
  constexpr int IMU_INT_ICM20948 = 32;
  constexpr int MAG_DRDY         = 40;   // QMC5883L

  // Peripherals
  constexpr int SAFETY_BTN       = 35;   // active-low, internal PU
  constexpr int BUZZER           = 26;
  constexpr int LED              = -1;   // TODO: on-FC LED on GPIO_SD_B1_11 (not exposed via Arduino); wire carrier LED_ACTIVITY or use external
  constexpr int ESC_CURRENT_ADC  = 41;   // 4-in-1 ESC shunt (future task #23)
  constexpr int VBAT_ADC         = 24;   // TODO: user should pick ADC pin based on carrier/divider wiring

  // CRSF / ELRS RX on Serial6 (pins 24/25) for Spin 1 board.
  #define CRSF_SERIAL Serial6
}
#endif

// ---- Battery / current monitoring ------------------------------------------
static constexpr float VBAT_DIVIDER_RATIO = 11.0f;   // e.g. 10k:1k divider -> ratio 11
static constexpr float VBAT_FILTER_BETA   = 0.05f;   // slow — battery doesn't change fast
static constexpr float VBAT_WARN_V        = 3.5f;    // per cell warning
static constexpr float VBAT_CUT_V         = 3.3f;    // per cell hard cutoff
static constexpr uint8_t BATTERY_CELL_COUNT = 4;     // 4S default

static constexpr float ESC_CURRENT_V_PER_A     = 0.04f;  // 40 mV/A typical
static constexpr float ESC_CURRENT_OFFSET_V    = 0.0f;   // ESC-specific zero offset
static constexpr float ESC_CURRENT_FILTER_BETA = 0.05f;

// Servo array size used by driver loops — computed from pins::SERVO above.
static constexpr size_t SERVO_COUNT = sizeof(pins::SERVO) / sizeof(pins::SERVO[0]);
