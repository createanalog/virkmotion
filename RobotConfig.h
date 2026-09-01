#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// ============================================================================
// RobotConfig.h — all the constants you tune to match Virk's actual build:
// link lengths, steps/rev, pins, motion limits, homing/calibration
// parameters, software limits, and the timing constants for the step ISR
// and the cartesian interpolator. ADJUST THESE to your own robot before
// flashing — the defaults here are just reasonable starting points.
// ============================================================================

#include <Arduino.h>

// ---------------- Arm parameters (ADJUST) ----------------
constexpr float L1_MM = 150.0f;   // length of the first link
constexpr float L2_MM = 150.0f;   // length of the second link

// Steps per motor revolution * microstepping, divided by the mechanical
// reduction of each joint (belt/gears). Adjust to your build.
constexpr float STEPS_PER_REV_J1 = 3200.0f; // e.g. 200 steps/rev * 16 microsteps
constexpr float STEPS_PER_REV_J2 = 3200.0f;
constexpr float STEPS_PER_MM_Z   = 80.0f;   // linear Z axis (leadscrew/belt), steps/mm

constexpr float STEPS_PER_RAD_J1 = STEPS_PER_REV_J1 / (2.0f * PI);
constexpr float STEPS_PER_RAD_J2 = STEPS_PER_REV_J2 / (2.0f * PI);

// ---------------- Pins — Wemos D1 R32 + standard CNC Shield V3 ----------------
// The Wemos D1 R32 puts the ESP32 in an Arduino Uno footprint, so a CNC
// Shield V3 (X/Y/Z/A driver sockets, GRBL-style pin layout) plugs in
// directly. The shield fixes WHICH Uno pin (D2, D5, D9, etc.) each signal
// lands on — you can't rewire that without cutting traces — so what
// actually varies per board is which ESP32 GPIO sits behind each Uno pin
// label. The mapping below is for the Wemos D1 R32 specifically; a
// different ESP32-in-Uno-clothing board could differ.
//
// Shield socket -> robot axis assignment used here:
//   X socket -> J1 (shoulder)      Y socket -> J2 (elbow)      Z socket -> Z (lift)
// The shield's 4th (A) axis socket is left unused for now — it shares D12/
// D13 with the shield's spindle-enable/spindle-dir pins and needs a jumper
// to run independently. Reserved for a future 4th joint (wrist rotation).
//
//   Signal          Shield/Uno pin   Wemos D1 R32 GPIO
//   X.STEP (J1)     D2               GPIO26
//   X.DIR  (J1)     D5               GPIO16
//   Y.STEP (J2)     D3               GPIO25
//   Y.DIR  (J2)     D6               GPIO27
//   Z.STEP (Z)      D4               GPIO17
//   Z.DIR  (Z)      D7               GPIO14
//   EN (shared)     D8               GPIO12  <-- see warning below
//   X-endstop (J1)  D9               GPIO13
//   Y-endstop (J2)  D10              GPIO5   <-- see warning below
//   Z-endstop (Z)   D11              GPIO23
//   A.STEP (spare)  D12              GPIO19  (unused, reserved for a 4th joint)
//   A.DIR  (spare)  D13              GPIO18  (unused, reserved for a 4th joint)
//
// !! IMPORTANT BOOT-TIME WARNING !!
// GPIO12 and GPIO5 are ESP32 "strapping pins", sampled once at power-on/
// reset to choose flash voltage (GPIO12) and SDIO boot config (GPIO5):
//  - GPIO12 (the shield's shared driver ENABLE, D8) must be LOW at boot,
//    or the ESP32 can select 1.8V flash mode and fail to boot/brown out.
//    Many CNC Shield V3 boards have a pull-up on the EN line so drivers
//    default to disabled — if yours does, and Virk won't boot with the
//    shield's drivers installed, that pull-up is almost certainly why.
//    A pull-down of a few kOhm from D8 to GND (or cutting the shield's own
//    pull-up, if present) fixes it. GPIO12 defaults LOW on its own if left
//    unconnected, so this only bites when something actively pulls it high.
//  - GPIO5 (Y-endstop, D10) wants to be HIGH at boot, which lines up fine
//    with INPUT_PULLUP + a normally-open switch — just don't power the
//    robot on with the Y limit switch physically pressed.
// Neither is likely to cause trouble in normal use once past boot, but
// they're worth knowing about if Virk ever fails to start with the shield
// attached.
//
// Also note: this firmware doesn't currently drive the shared ENABLE pin
// (StepperAxis has no enable-pin concept) — for now, wire/jumper D8 to
// stay LOW so the drivers are always enabled, or ask me to add ENABLE_PIN
// handling to setup() if you'd rather control it from code.
constexpr uint8_t J1_STEP_PIN = 26;
constexpr uint8_t J1_DIR_PIN  = 16;
constexpr uint8_t J2_STEP_PIN = 25;
constexpr uint8_t J2_DIR_PIN  = 27;
constexpr uint8_t Z_STEP_PIN  = 17;
constexpr uint8_t Z_DIR_PIN   = 14;

constexpr uint8_t ENABLE_PIN  = 12; // shield's shared driver ENABLE (D8) — see warning above; not yet driven by firmware

// Limit switches for homing (calibration), wired to the shield's fixed
// X/Y/Z endstop headers. Use INPUT_PULLUP and are considered "triggered"
// when they read LOW (normally-open switch to GND).
constexpr uint8_t J1_LIMIT_PIN = 13;
constexpr uint8_t J2_LIMIT_PIN = 5;
constexpr uint8_t Z_LIMIT_PIN  = 23;

// ---------------- Motion limits (ADJUST) ----------------
constexpr float J1_VMAX_SPS = 4000.0f;   // steps/s
constexpr float J1_AMAX_SPS2 = 8000.0f;  // steps/s^2
constexpr float J2_VMAX_SPS = 4000.0f;
constexpr float J2_AMAX_SPS2 = 8000.0f;
constexpr float Z_VMAX_SPS = 2000.0f;
constexpr float Z_AMAX_SPS2 = 4000.0f;

// ---------------- Calibration (homing with switches) ----------------
// Known position (angle/height) at the EXACT point where each switch
// triggers. Doesn't have to be 0 — put the real value for your build.
constexpr float J1_HOMING_ANGLE_DEG = 0.0f;
constexpr float J2_HOMING_ANGLE_DEG = 0.0f;
constexpr float Z_HOMING_MM = 0.0f;

// Direction (+1 or -1) to move in order to APPROACH each axis's switch.
// Depends on where the sensor is physically mounted — adjust to your robot
// (if it homes "backwards", flip the sign).
constexpr int HOMING_DIR_J1 = -1;
constexpr int HOMING_DIR_J2 = -1;
constexpr int HOMING_DIR_Z  = -1;

// Homing speeds as a fraction of each axis's normal VMAX: fast for the
// first approach, slow for the precision re-approach.
constexpr float HOMING_FAST_FRACTION = 0.25f;
constexpr float HOMING_SLOW_FRACTION = 0.03f;
constexpr float HOMING_BACKOFF_FRACTION = 0.10f;

constexpr long HOMING_BACKOFF_EXTRA_STEPS = 200; // beyond releasing the switch, before the slow re-approach
constexpr long HOMING_CLEARANCE_STEPS = 100;      // final clearance from the switch after calibrating, so it isn't left pressed
constexpr unsigned long HOMING_TIMEOUT_MS = 15000; // per phase; if exceeded, aborts with an error (protects against a broken/miswired switch)

// ---------------- Software limits ----------------
// Total safe MECHANICAL travel range of each axis (everything the robot can
// move without hitting anything), and a safety margin to stay away from the
// homing switch during normal operation (avoids bumping/triggering it on
// every move near that end). ADJUST to your actual build — these are just
// reasonable starting values.
constexpr float J1_TRAVEL_RANGE_DEG = 300.0f;
constexpr float J1_HOMING_MARGIN_DEG = 3.0f;
constexpr float J2_TRAVEL_RANGE_DEG = 300.0f;
constexpr float J2_HOMING_MARGIN_DEG = 3.0f;
constexpr float Z_TRAVEL_RANGE_MM = 100.0f;
constexpr float Z_HOMING_MARGIN_MM = 2.0f;

// Each axis's switch sits at the end its HOMING_DIR points toward; the
// "switch-side" limit is computed by backing off HOMING_MARGIN from the
// known homing position, and the "far-side" limit by traveling the full
// TRAVEL_RANGE toward the other end. Which of the two is the min and which
// is the max depends on the sign of HOMING_DIR, so it's resolved with
// min/max instead of assuming an order.
constexpr float J1_LIMIT_A_DEG = J1_HOMING_ANGLE_DEG - HOMING_DIR_J1 * J1_HOMING_MARGIN_DEG;
constexpr float J1_LIMIT_B_DEG = J1_HOMING_ANGLE_DEG - HOMING_DIR_J1 * J1_TRAVEL_RANGE_DEG;
constexpr float J1_SOFT_MIN_DEG = (J1_LIMIT_A_DEG < J1_LIMIT_B_DEG) ? J1_LIMIT_A_DEG : J1_LIMIT_B_DEG;
constexpr float J1_SOFT_MAX_DEG = (J1_LIMIT_A_DEG > J1_LIMIT_B_DEG) ? J1_LIMIT_A_DEG : J1_LIMIT_B_DEG;

constexpr float J2_LIMIT_A_DEG = J2_HOMING_ANGLE_DEG - HOMING_DIR_J2 * J2_HOMING_MARGIN_DEG;
constexpr float J2_LIMIT_B_DEG = J2_HOMING_ANGLE_DEG - HOMING_DIR_J2 * J2_TRAVEL_RANGE_DEG;
constexpr float J2_SOFT_MIN_DEG = (J2_LIMIT_A_DEG < J2_LIMIT_B_DEG) ? J2_LIMIT_A_DEG : J2_LIMIT_B_DEG;
constexpr float J2_SOFT_MAX_DEG = (J2_LIMIT_A_DEG > J2_LIMIT_B_DEG) ? J2_LIMIT_A_DEG : J2_LIMIT_B_DEG;

constexpr float Z_LIMIT_A_MM = Z_HOMING_MM - HOMING_DIR_Z * Z_HOMING_MARGIN_MM;
constexpr float Z_LIMIT_B_MM = Z_HOMING_MM - HOMING_DIR_Z * Z_TRAVEL_RANGE_MM;
constexpr float Z_SOFT_MIN_MM = (Z_LIMIT_A_MM < Z_LIMIT_B_MM) ? Z_LIMIT_A_MM : Z_LIMIT_B_MM;
constexpr float Z_SOFT_MAX_MM = (Z_LIMIT_A_MM > Z_LIMIT_B_MM) ? Z_LIMIT_A_MM : Z_LIMIT_B_MM;

// ---------------- Control ISR frequency ----------------
constexpr uint32_t ISR_FREQ_HZ = 20000;  // every 50us we check if we need to issue motor steps
constexpr float ISR_DT_S = 1.0f / ISR_FREQ_HZ;

// ---------------- Cartesian interpolation (G1, G2, G3 moves) ----------------
constexpr float LINEAR_VMAX_MMPS = 80.0f;
constexpr float LINEAR_AMAX_MMPS2 = 200.0f;

constexpr uint32_t INTERP_FREQ_HZ = 400;
constexpr uint32_t INTERP_PERIOD_US = 1000000UL / INTERP_FREQ_HZ;

// ---------------- Move queue and corner velocity ----------------
// How much "corner cutting" is allowed between two consecutive segments, in
// mm: higher = can take tight corners faster, but the tip of the arm will
// deviate more from the ideal vertex. Same concept as "junction deviation"
// in GRBL/Marlin-style 3D printer/CNC firmware.
constexpr float JUNCTION_DEVIATION_MM = 0.02f;
constexpr int QUEUE_SIZE = 32;  // gcode waiting queue size

#endif
