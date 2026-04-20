// dRehmFlight — Restructured
// Thin entry point. setup() wires drivers; loop() ticks the scheduler.
// All control math, sensor drivers, and airframe mixing live under src/.

#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "src/scheduler.h"

// ---- sensor drivers (compile-time selected) ---------------------------------

#if defined(IMU_MPU6050)
  #include "src/sensors/imu_mpu6050.h"
  static ImuMpu6050 imu;
#elif defined(IMU_MPU9250)
  #include "src/sensors/imu_mpu9250.h"
  static ImuMpu9250 imu;
#elif defined(IMU_BMI270)
  #include "src/sensors/imu_bmi270.h"
  static ImuBmi270 imu;
#elif defined(IMU_ICM20948)
  #include "src/sensors/imu_icm20948.h"
  static ImuIcm20948 imu;
#elif defined(IMU_ICM42688)
  #include "src/sensors/imu_icm42688.h"
  static ImuIcm42688 imu;
#elif defined(IMU_MULTI)
  // Multi-IMU fusion — edit this block to match the chips on your board.
  // Median-of-3 voting needs >=3 sub-IMUs; N=2 averages; N=1 passes through.
  #include "src/sensors/imu_multi.h"
  #include "src/sensors/imu_bmi270.h"
  #include "src/sensors/imu_icm20948.h"
  #include "src/sensors/imu_icm42688.h"
  static ImuBmi270   sub_imu_0;
  static ImuIcm20948 sub_imu_1;
  static ImuIcm42688 sub_imu_2;
  static IMU* sub_imus[] = { &sub_imu_0, &sub_imu_1, &sub_imu_2 };
  static MultiImu imu(sub_imus, sizeof(sub_imus) / sizeof(sub_imus[0]));
#else
  #error "No IMU selected in config.h"
#endif

#if defined(MAG_QMC5883L)
  #include "src/sensors/mag_qmc5883l.h"
  static MagQmc5883l mag;
  #define HAVE_STANDALONE_MAG 1
#else
  #define HAVE_STANDALONE_MAG 0
#endif

#if defined(BARO_BMP585)
  #include "src/sensors/baro_bmp585.h"
  static BaroBmp585 baro;
  #define HAVE_BARO 1
#else
  #define HAVE_BARO 0
#endif

// ---- RC driver --------------------------------------------------------------

#if defined(RC_PWM)
  #include "src/rc/rc_pwm.h"
  static RcPwm rc;
#elif defined(RC_PPM)
  #include "src/rc/rc_ppm.h"
  static RcPpm rc;
#elif defined(RC_SBUS)
  #include "src/rc/rc_sbus.h"
  static RcSbus rc;
#elif defined(RC_DSM)
  #include "src/rc/rc_dsm.h"
  static RcDsm rc;
#elif defined(RC_CRSF)
  #include "src/rc/rc_crsf.h"
  static RcCrsf rc;
  #define HAVE_TELEMETRY 1
#else
  #error "No RC driver selected in config.h"
#endif

#ifndef HAVE_TELEMETRY
  #define HAVE_TELEMETRY 0
#endif

// ---- actuators + control ----------------------------------------------------

#include "src/actuators/servos.h"
#include "src/control/attitude.h"
#include "src/control/pid.h"
#include "src/control/mixer.h"
#include "src/control/arming.h"
#include "src/control/flight_mode.h"
#include "src/filters/rpm_filter.h"
#include "src/telemetry/msp.h"
#include "src/telemetry/blackbox.h"
#include "src/sensors/battery.h"
#include "src/storage/config_store.h"
#include "src/peripherals/buzzer.h"
#include "src/peripherals/led.h"

#if defined(MOTORS_ONESHOT125)
  #include "src/actuators/motors_oneshot125.h"
  static MotorsOneShot125 motors;
#elif defined(MOTORS_DSHOT)
  #include "src/actuators/motors_dshot.h"
  static MotorsDShot motors;
#else
  #error "No motor protocol selected in config.h"
#endif

static Servos servos;
static State  state;
static Scheduler scheduler;

// ---- tasks ------------------------------------------------------------------
// The scheduler's first task is the control task — it runs at CONTROL_RATE_HZ
// and is the sole writer of motor/servo commands. Lower-rate tasks only touch
// their own State fields.

static void task_control(State& s) {
  rc.update(s);
  arming::update(s);
  imu.update_calibrated(s);         // raw gyro/accel with bias removed, no LP yet
  rpm_filter::apply(s);             // notches before LP — uses motor_erpm[] from last write
  imu.update_filtered(s);           // soft LP (B_GYRO / B_ACCEL) after notches
  attitude::madgwick(s);
  pid::get_desired_state(s);
  flight_mode::update(s);           // CH6 -> s.mode (ACRO / ANGLE / HORIZON)
  flight_mode::dispatch(s);         // runs the right pid::control_* for the mode
  mixer::mix(s);
  if (arming::should_cut(s)) {
    motors.cut(s);                  // !armed || CH5>1500 || failsafe -> zero outputs
  } else {
    motors.scale(s);
  }
  servos.scale(s);
  motors.write(s);                  // bidir DShot captures eRPM for next tick
  servos.write(s);
  if (s.armed) blackbox::log_frame(s);
}

#if HAVE_STANDALONE_MAG
static void task_mag(State& s) { mag.update(s); }
#endif

#if HAVE_BARO
static void task_baro(State& s) { baro.update(s); }
#endif

#if HAVE_TELEMETRY
static void task_telem(State& s) { rc.send_telemetry(s); }
#endif

static void task_msp(State& s)     { msp::poll(s); }
static void task_blackbox(State& s){ blackbox::flush(); }
static void task_battery(State& s) { battery::update(s); }

// Alerts dispatcher — picks buzzer/LED patterns based on State and priority.
static void task_alerts(State& s) {
  static bool was_armed = false;

  // Edge-triggered ack beeps
  if (s.armed && !was_armed) buzzer::play(buzzer::Pattern::ARMED);
  if (!s.armed && was_armed) buzzer::play(buzzer::Pattern::DISARMED);
  was_armed = s.armed;

  // Continuous alerts in priority order (failsafe > critical bat > low bat > idle)
  if (s.rc_failsafe) {
    buzzer::play(buzzer::Pattern::FAILSAFE);
    led::play(led::Pattern::FAST_BLINK);
  } else if (s.battery_valid && battery::is_critical(s)) {
    buzzer::play(buzzer::Pattern::CRITICAL_BATTERY);
    led::play(led::Pattern::FAST_BLINK);
  } else if (s.battery_valid && battery::is_low(s)) {
    buzzer::play(buzzer::Pattern::LOW_BATTERY);
    led::play(s.armed ? led::Pattern::SOLID : led::Pattern::DOUBLE_BLINK);
  } else {
    led::play(s.armed ? led::Pattern::SOLID : led::Pattern::SLOW_BLINK);
    // Let any currently-playing one-shot finish; don't reset to NONE here
    // or we'd clobber the arm/disarm ack.
  }

  buzzer::update();
  led::update();
}

static Task tasks[] = {
  { 1000000UL / CONTROL_RATE_HZ, 0, task_control, "ctrl" },
#if HAVE_STANDALONE_MAG
  { 1000000UL / MAG_RATE_HZ,     0, task_mag,     "mag"  },
#endif
#if HAVE_BARO
  { 1000000UL / BARO_RATE_HZ,    0, task_baro,    "baro" },
#endif
#if HAVE_TELEMETRY
  { 1000000UL / TELEM_RATE_HZ,   0, task_telem,   "tlm"  },
#endif
  { 1000000UL / 100,             0, task_msp,      "msp"  },  // Configurator
  { 1000000UL / 5,               0, task_blackbox, "bb"   },  // SD flush
  { 1000000UL / 50,              0, task_battery,  "batt" },  // VBAT + current
  { 1000000UL / 50,              0, task_alerts,   "alrt" },  // buzzer + LED
};
static constexpr uint8_t NUM_TASKS = sizeof(tasks) / sizeof(tasks[0]);

// ---- setup / loop -----------------------------------------------------------

static void fatal(const char* msg) {
  Serial.println(msg);
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
}

void setup() {
  Serial.begin(500000);
  delay(500);

  config_store::begin();            // must precede imu/pid — their cal + gains read from here
  battery::begin();
  buzzer::begin();
  led::begin();

  if (!imu.begin()) fatal("IMU init failed");
#if HAVE_STANDALONE_MAG
  if (!mag.begin()) fatal("MAG init failed");
#endif
#if HAVE_BARO
  if (!baro.begin()) fatal("BARO init failed");
#endif
  if (!rc.begin())  fatal("RC init failed");

  motors.begin();
  servos.begin();
  arming::begin();
  pid::reset_integrators();
  rpm_filter::init();
  msp::begin();
  blackbox::begin();                // silently disabled if no SD / Teensy 4.0
  motors.arm();                     // ESC init pulse sequence — motors stay cut
                                    // until arming::update() sets s.armed via sticks

  scheduler.begin(tasks, NUM_TASKS, CONTROL_RATE_HZ);
}

void loop() {
  scheduler.tick(state);
}
