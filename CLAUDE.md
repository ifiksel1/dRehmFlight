# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Two parallel variants of the dRehmFlight flight controller live here:

1. **`Versions/dRehmFlight_Teensy_BETA_1.{0,1,2,3}/`** — the original upstream. Single monolithic `.ino`, flat, no abstractions, educational intent. Treat as reference. Beta 1.3 is current upstream; only edit older betas if explicitly asked.
2. **`Versions/dRehmFlight_Restructured/`** — a clean rewrite under modular abstractions (one `.h/.cpp` per driver, abstract base classes, a scheduler, a shared `State` struct). **Default to editing this when the user asks about "the firmware" unless they specifically say Beta 1.x.**

The restructure is the active development target. It reads the same flight-control math as Beta 1.3 — Madgwick, `controlANGLE/RATE/ANGLE2`, same PID heritage — but everything around the math is rearranged so adding sensors, RC protocols, ESC protocols, control features, and boards is a local change.

## Restructured layout

```
Versions/dRehmFlight_Restructured/
├── dRehmFlight_Restructured.ino   # thin entry: factory #ifdefs, setup(), loop()
├── config.h                       # all compile-time selectors, tunables, pin defs
├── state.h                        # shared State struct (single source of truth for tick data)
└── src/
    ├── scheduler.{h,cpp}          # multi-rate cooperative runner (first task = control, wrap-safe)
    ├── sensors/
    │   ├── imu.{h,cpp}            # abstract IMU: readRaw() virtual, update_calibrated + update_filtered split
    │   ├── imu_{mpu6050,mpu9250,bmi270,icm20948,icm42688,multi}.{h,cpp}
    │   ├── baro.{h,cpp}           # abstract Baro: pressure + altitude derivation
    │   ├── baro_bmp585.{h,cpp}
    │   ├── mag.{h,cpp}            # abstract Mag with cal + LP filter
    │   ├── mag_qmc5883l.{h,cpp}
    │   └── battery.{h,cpp}        # VBAT + ESC current + mAh integration
    ├── rc/
    │   ├── rc.{h,cpp}             # abstract Rc with failsafe
    │   └── rc_{pwm,ppm,sbus,dsm,crsf}.{h,cpp}
    ├── actuators/
    │   ├── motors.h               # abstract Motors
    │   ├── motors_oneshot125.{h,cpp}
    │   ├── motors_dshot.{h,cpp}   # bidir DShot600 with eRPM telemetry
    │   └── servos.{h,cpp}
    ├── control/
    │   ├── attitude.{h,cpp}       # Madgwick (branches on mag_valid)
    │   ├── pid.{h,cpp}            # control_angle / control_rate / control_angle2
    │   ├── mixer.{h,cpp}          # per-airframe customization point
    │   ├── flight_mode.{h,cpp}    # CH6 3-pos → ACRO / ANGLE / HORIZON dispatch
    │   └── arming.{h,cpp}         # stick-arm + safety-button interlock + throttle-cut gate
    ├── filters/
    │   └── rpm_filter.{h,cpp}     # Betaflight-style per-motor biquad notches
    ├── telemetry/
    │   ├── msp.{h,cpp}            # MultiWii Serial Protocol v1+v2 (Betaflight Configurator compat)
    │   └── blackbox.{h,cpp}       # Teensy 4.1 SD logging, fixed 132-byte binary frames
    ├── peripherals/
    │   ├── buzzer.{h,cpp}         # non-blocking tone() pattern player
    │   └── led.{h,cpp}            # non-blocking digitalWrite blink patterns
    ├── storage/
    │   └── config_store.{h,cpp}   # EEPROM-backed runtime-editable gains + calibration (versioned + CRC32)
    └── {MPU6050,MPU9250,SBUS,DSMRX,TFMPlus}/   # vendored third-party drivers
```

## Build and verify

Use `arduino-cli` from `/home/dev/.local/bin` (already installed on this machine, Teensy core 1.60.0 installed). From the sketch directory:

```
export PATH=/home/dev/.local/bin:$PATH
arduino-cli compile --fqbn teensy:avr:teensy40 --output-dir /tmp/build
# or teensy41 for SD-card features (blackbox only activates on Teensy 4.1)
```

A successful build produces `.elf` and `.hex` in the output dir. Running the Teensy post-link tools (`teensy_secure`, `teensy_post_compile`, `teensy_size`, etc.) fails on this Jetson with GLIBC 2.33 missing — those binaries have been stubbed with `#!/bin/sh\nexit 0` shims. Originals are preserved as `.orig` next to each. This means **this machine can compile+link but not flash** — move the resulting `.hex` to a real dev machine for flashing, or revert the stubs if/when a compatible glibc is available.

There is no test suite. Verification is compile + hardware in the loop.

## Architecture essentials

### The control task

`task_control()` in the `.ino` runs at `CONTROL_RATE_HZ` (default 1500; see config.h comment for the DShot600 budget math). Pipeline:

1. `rc.update(s)` — pulse widths → `s.rc[16]` with failsafe
2. `arming::update(s)` — sticky arm once CH5<1500 AND CH1<1050 AND safety not pressed
3. `imu.update_calibrated(s)` — raw → bias-corrected gyro/accel/mag (no LP yet)
4. `rpm_filter::apply(s)` — notch out motor fundamentals using `s.motor_erpm` from last tick
5. `imu.update_filtered(s)` — soft LP (B_GYRO / B_ACCEL) after notches
6. `attitude::madgwick(s)` — quaternion + euler attitude
7. `pid::get_desired_state(s)` — sticks → setpoints
8. `flight_mode::update(s)` / `flight_mode::dispatch(s)` — CH6 picks ACRO/ANGLE/HORIZON
9. `mixer::mix(s)` — PID outputs → motor + servo scaled commands (edit for your airframe)
10. `arming::should_cut(s)` gate → `motors.cut(s)` or `motors.scale(s)`
11. `motors.write(s)` — OneShot125 bit-bang or DShot (reads back eRPM inline)
12. `servos.write(s)`
13. `if (s.armed) blackbox::log_frame(s)` — 132-byte binary frame to SD (Teensy 4.1 only)

Lower-rate tasks (baro 100 Hz, mag 100 Hz, CRSF telemetry 50 Hz, MSP 100 Hz, blackbox flush 5 Hz, battery 50 Hz, alerts 50 Hz) run cooperatively between control ticks.

### config.h vs config_store::data

- `config.h` — compile-time **defaults** for gains, calibration, filter betas, pin assignments, protocol selectors. `constexpr` values.
- `config_store::data` (from `src/storage/config_store.h`) — **runtime-editable** copy of gains + calibration persisted to EEPROM. Loaded at boot; falls back to config.h defaults + auto-saves on first boot or CRC fail.
- **`pid::control_*` and `IMU::update_calibrated` read from `config_store::data`, NOT from the `KP_*`/`ACC_ERR_*` constants.** If you're changing how a gain or offset is read or written, edit through `config_store::data`. The `constexpr` in `config.h` is only the seed value.
- **MSP write-through model:** `MSP_SET_PID` edits `config_store::data` live (takes effect next tick). Persistence happens only on `MSP_EEPROM_WRITE`. Betaflight Configurator's "Save" button triggers both in order.

### The State struct

`state.h` defines one big struct owned by the top-level `.ino` and passed by reference to every subsystem. Every piece of per-tick data lives here: sensor readings (`gyro`, `accel`, `mag`, `pressure_pa`, `altitude_m`, `vbat_v`, `current_a`, `mah_consumed`), attitude (`q`, `euler`), RC channels, setpoints, PID outputs, motor/servo commands, `motor_erpm[]` from bidir DShot, armed state, flight mode enum. **Keep it this way**: don't move fields into file-static globals in modules — State is the contract.

### Board selector

`config.h` has two pin mappings behind `#if defined(BOARD_TEENSY41_DEFAULT)` vs the default `BOARD_CUSTOM_RT1062` (the Spin 1 custom carrier, documented in `pin_allocation.xlsx`). The board selector controls:
- Motor, servo, IMU CS, buzzer, LED, safety-button, ESC current sense pins
- Which UART the CRSF driver compiles against (`CRSF_SERIAL` macro — Serial6 on Spin 1, Serial5 on default Teensy 4.1)
- Whether the safety-button interlock is active (default board has no pin, gate is compiled out)

## Conventions

- **Tight code, minimal comments.** Style reference: `src/sensors/imu_mpu6050.cpp`. No docstrings, no verbose block comments. Only comment the non-obvious: register choices, protocol quirks, cycle-count timing, flight-critical invariants.
- **Each optional driver `.cpp` guards its body** with `#if defined(SELECTOR_MACRO)` so Arduino's "compile every file in the tree" doesn't break when the driver's vendored library isn't present. Look at `imu_bmi270.cpp` for the pattern.
- **Relative include paths from `src/<subdir>/`**: `"../../config.h"`, `"../../state.h"`. Scheduler at `src/scheduler.h` uses `"../state.h"`. When adding new files, verify these don't drift.
- **Abstract-base-plus-concrete pattern** for every sensor/protocol category (IMU, Baro, Mag, Rc, Motors). Base class handles the shared pipeline (calibration, LP filter, failsafe, altitude derivation); subclasses implement `begin()` + `readRaw()` only. Adding a new chip or protocol = one new `.h/.cpp` pair plus one `#elif` in the factory block in the `.ino`.
- **Mixer is the per-airframe customization point.** `src/control/mixer.cpp` currently has the Beta 1.3 default quad mapping. Any VTOL / tricopter / bicopter / etc. user overwrites this function. Don't add airframe variants as subclasses — keep it editable in-place.
- **Don't silently retune filters.** `B_GYRO`/`B_ACCEL`/`B_MAG` and the PID gains are calibrated for specific chip + loop-rate combinations. If you change `CONTROL_RATE_HZ`, the LP cutoffs shift and flight feel changes. Call this out explicitly.

## Library vendoring

The `src/` tree expects third-party driver libraries to live at `src/<ChipName>/`. Beta 1.3 ships `MPU6050`, `MPU9250`, `SBUS`, `DSMRX`, `TFMPlus` — these were copied into the restructured tree and work out of the box. For the new chips, **the user must drop in**:

- `src/BMI270/` (assume Arduino Nicla Sense style API)
- `src/ICM20948/` (assume SparkFun `ICM_20948_SPI`)
- `src/ICM42688/` (assume Bolder Flight `ICM42688`)
- `src/BMP585/` (assume SparkFun BMP581/BMP585)
- `src/QMC5883L/` (assume mprograms/QMC5883LCompass-style)

Each new-chip `.cpp` has a `// ASSUMES:` comment naming the expected API. If the vendored library has a different signature, adjust the driver — don't change the abstract interface.

## Future work

Many features are staged as tasks. Before starting anything big, run `TaskList` to see what's pending. High-value open items include: staged timed failsafe (#25), rates curves + expo (#26), TPA + anti-gravity + I-term relax (#27, #28), airmode (#29), crash auto-disarm (#30), gyro temp compensation (#31), OSD (#35), and the big one — companion-computer + 15-state EKF for indoor/vision flight (#11).

## Verification is your responsibility

Every existing module has been **compiled** against Teensy core 1.60.0 but nothing has been **flown or bench-tested**. Most latent bugs will only surface on silicon. When reporting work done on a driver, ring-buffer, or timing-critical path: say so explicitly rather than implying it works.
