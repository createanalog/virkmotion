// ============================================================================
// SCARA controller built from scratch — ESP32 + stepper drivers (STEP/DIR)
// Kinematics (forward/inverse) + trapezoidal velocity profile + move queue
// with "corner" velocity so consecutive segments don't fully stop between
// each other + pulse generation via phase accumulator through a HW timer.
//
// Created by automaticartisan, with the assistance of Claude Sonnet 5
// (Anthropic). Licensed under the MIT License — see LICENSE.
// ============================================================================
//
// ADJUST THESE PARAMETERS TO YOUR ROBOT BEFORE USING IT:
//   - Link lengths (L1_MM, L2_MM)
//   - Steps per revolution of each motor and gear reduction ratio (if any)
//   - STEP/DIR pins for each axis
//   - Maximum velocity and acceleration limits
//
// Serial commands (115200 baud), one per line. G0/G1/G2/G3 get QUEUED
// (not executed immediately); they get dispatched in order as soon as the
// previous segment finishes.
//   G0 X<mm> Y<mm> Z<mm>            -> fast move, synchronized in joint
//                                       space (curved path, always fully
//                                       stops before/after)
//   G1 X<mm> Y<mm> Z<mm> F<mm/s>    -> real LINEAR move (straight line)
//   G2 X<mm> Y<mm> Z<mm> I<mm> J<mm> F<mm/s>  -> real CLOCKWISE arc, I/J
//                                       are the center relative to the start
//   G3 ... (same as G2 but counter-clockwise)
//   CALIBRATE                        -> homing with switches on J1, J2 and
//                                       Z: finds each switch and sets the
//                                       known position at that point (see below)
//   SETPOS X<mm> Y<mm> Z<mm>         -> redefines the current cartesian
//                                       position WITHOUT moving motors (for
//                                       after repositioning the arm by hand,
//                                       or fine-tuning a calibration offset)
//   CLEAR                            -> empties the queue (doesn't abort the
//                                       segment already in progress)
//   HOME                             -> defines the current position as the origin
//   ?                                 -> reports position and queue status
//
// ============================================================================

#include "Kinematics.h"
#include "TrapProfile.h"
#include "StepperAxis.h"

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

// ---------------- Pins (ADJUST to your wiring) ----------------
constexpr uint8_t J1_STEP_PIN = 25;
constexpr uint8_t J1_DIR_PIN  = 26;
constexpr uint8_t J2_STEP_PIN = 27;
constexpr uint8_t J2_DIR_PIN  = 14;
constexpr uint8_t Z_STEP_PIN  = 12;
constexpr uint8_t Z_DIR_PIN   = 13;

// Limit switches for homing (calibration). Use INPUT_PULLUP and are
// considered "triggered" when they read LOW (normally-open switch to GND).
// WATCH OUT: on ESP32, GPIO34-39 do NOT have an internal pull-up resistor —
// if you use one of those, add an external resistor or pick a different pin.
constexpr uint8_t J1_LIMIT_PIN = 32;
constexpr uint8_t J2_LIMIT_PIN = 33;
constexpr uint8_t Z_LIMIT_PIN  = 4;

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
constexpr uint32_t ISR_FREQ_HZ = 20000;
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
constexpr int QUEUE_SIZE = 8;

ScaraKinematics kinematics(L1_MM, L2_MM);
StepperAxis axisJ1, axisJ2, axisZ;

hw_timer_t *isrTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Controller's known "logical" cartesian position (mm); updated when each
// move *starts* (target position), not in real time.
Pose currentTargetPose = {L1_MM + L2_MM * 0.0f, 0.0f, 0.0f}; // see setup()

// Cartesian velocity (mm/s) the last dispatched G1/G2/G3 segment ended at.
// Used as the starting velocity of the next segment, so it doesn't need to
// fully stop at the junction if the geometry allows it. Anything that
// breaks continuity (G0, error, empty queue) resets it to 0.
float carryVelocity = 0.0f;

void IRAM_ATTR onTimerISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  axisJ1.update(ISR_DT_S);
  axisJ2.update(ISR_DT_S);
  axisZ.update(ISR_DT_S);
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ---------------- Minimal 3D vectors (only what's needed for directions) ----------------
struct Vec3 { float x, y, z; };

Vec3 normalize3(Vec3 v) {
  float len = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  if (len < 1e-6f) return {0, 0, 0};
  return {v.x / len, v.y / len, v.z / len};
}
float dot3(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// ---------------- Software limits ----------------
// Checks that an inverse-kinematics solution is WITHIN each axis's safe
// range (not just that it's mathematically reachable). Applied everywhere
// that used to only look at JointAngles::reachable — CALIBRATE is the only
// routine that intentionally skips this check, because it needs to be able
// to reach the switch to home.
bool jointsWithinSoftLimits(const JointAngles &j) {
  float j1Deg = degrees(j.theta1);
  float j2Deg = degrees(j.theta2);
  if (j1Deg < J1_SOFT_MIN_DEG || j1Deg > J1_SOFT_MAX_DEG) return false;
  if (j2Deg < J2_SOFT_MIN_DEG || j2Deg > J2_SOFT_MAX_DEG) return false;
  if (j.z < Z_SOFT_MIN_MM || j.z > Z_SOFT_MAX_MM) return false;
  return true;
}

// Inverse kinematics + combined check: reachable AND within software
// limits. Centralizes the error message so it's consistent across the code.
bool solveAndCheckLimits(const Pose &target, ElbowConfig elbow, JointAngles &outJoints) {
  outJoints = kinematics.inverse(target, elbow);
  if (!outJoints.reachable) {
    Serial.println("ERROR: punto fuera del área de trabajo (geometría del brazo)");
    return false;
  }
  if (!jointsWithinSoftLimits(outJoints)) {
    Serial.println("ERROR: punto fuera de los límites de software (rango seguro configurado)");
    return false;
  }
  return true;
}

// ---------------- G0: joint-space synchronized move ----------------
// Always starts and ends at rest (breaks cartesian velocity continuity).
// Simpler/cheaper, but the tip of the arm does NOT follow a straight line
// between origin and destination.
bool moveJointSyncTo(const Pose &target, ElbowConfig elbow = ELBOW_UP) {
  JointAngles targetJoints;
  if (!solveAndCheckLimits(target, elbow, targetJoints)) {
    return false;
  }

  long targetStepsJ1 = lroundf(targetJoints.theta1 * STEPS_PER_RAD_J1);
  long targetStepsJ2 = lroundf(targetJoints.theta2 * STEPS_PER_RAD_J2);
  long targetStepsZ  = lroundf(targetJoints.z * STEPS_PER_MM_Z);

  long deltaJ1 = targetStepsJ1 - axisJ1.getPositionSteps();
  long deltaJ2 = targetStepsJ2 - axisJ2.getPositionSteps();
  long deltaZ  = targetStepsZ  - axisZ.getPositionSteps();

  axisJ1.prepareMove(deltaJ1, J1_VMAX_SPS, J1_AMAX_SPS2);
  axisJ2.prepareMove(deltaJ2, J2_VMAX_SPS, J2_AMAX_SPS2);
  axisZ.prepareMove(deltaZ, Z_VMAX_SPS, Z_AMAX_SPS2);

  float tMaster = max(axisJ1.getNaturalTime(), max(axisJ2.getNaturalTime(), axisZ.getNaturalTime()));

  if (tMaster <= 0.0f) {
    currentTargetPose = target;
    return true; // already there
  }

  portENTER_CRITICAL(&timerMux);
  axisJ1.rescaleAndStart(tMaster);
  axisJ2.rescaleAndStart(tMaster);
  axisZ.rescaleAndStart(tMaster);
  portEXIT_CRITICAL(&timerMux);

  currentTargetPose = target;
  return true;
}

// ---------------- G1/G2/G3: real cartesian interpolation ----------------

enum PathType { PATH_LINE, PATH_ARC };

struct CartesianMoveState {
  bool active = false;
  PathType type = PATH_LINE;
  Pose start, end;
  ElbowConfig elbow = ELBOW_UP;
  TrapProfile pathProfile;
  float totalLength = 0.0f;
  unsigned long moveStartMicros = 0;
  unsigned long lastTickMicros = 0;
  long lastCmdStepsJ1 = 0, lastCmdStepsJ2 = 0, lastCmdStepsZ = 0;
  bool warnedSaturation = false;

  // PATH_ARC only:
  Pose center;
  float radius = 0.0f;
  float startAngle = 0.0f;
  float deltaAngle = 0.0f;
};
CartesianMoveState cMove;

enum MoveCmdType { CMD_G0, CMD_G1, CMD_G2, CMD_G3 };
struct QueuedMove {
  MoveCmdType type;
  Pose target;
  float feedRate = -1.0f;   // <=0 means "use LINEAR_VMAX_MMPS"
  float offsetI = 0.0f, offsetJ = 0.0f; // G2/G3 only
};

QueuedMove moveQueue[QUEUE_SIZE];
int queueHead = 0, queueTail = 0, queueCount = 0;

bool queuePush(const QueuedMove &m) {
  if (queueCount >= QUEUE_SIZE) return false;
  moveQueue[queueTail] = m;
  queueTail = (queueTail + 1) % QUEUE_SIZE;
  queueCount++;
  return true;
}
bool queuePeek(int offset, QueuedMove &out) {
  if (offset >= queueCount) return false;
  out = moveQueue[(queueHead + offset) % QUEUE_SIZE];
  return true;
}
bool queuePop(QueuedMove &out) {
  if (queueCount == 0) return false;
  out = moveQueue[queueHead];
  queueHead = (queueHead + 1) % QUEUE_SIZE;
  queueCount--;
  return true;
}
void queueClear() { queueHead = queueTail = queueCount = 0; }

float feedRateOrDefault(float feedRate) {
  return (feedRate > 0.0f) ? min(feedRate, LINEAR_VMAX_MMPS) : LINEAR_VMAX_MMPS;
}

// Point on the path (line or arc) for a fraction 0..1 traveled.
Pose poseAtFraction(const CartesianMoveState &m, float frac) {
  Pose p;
  if (m.type == PATH_LINE) {
    p.x = m.start.x + (m.end.x - m.start.x) * frac;
    p.y = m.start.y + (m.end.y - m.start.y) * frac;
  } else {
    float angle = m.startAngle + m.deltaAngle * frac;
    p.x = m.center.x + m.radius * cosf(angle);
    p.y = m.center.y + m.radius * sinf(angle);
  }
  p.z = m.start.z + (m.end.z - m.start.z) * frac; // helix if type==PATH_ARC and start.z!=end.z
  return p;
}

// Direction of travel (unit tangent) of the path at a given fraction, via
// numerical differentiation — works the same for a line and an arc.
Vec3 pathTangentAtFraction(const CartesianMoveState &m, float frac) {
  const float EPS = 0.001f;
  float f0 = constrain(frac - EPS, 0.0f, 1.0f);
  float f1 = constrain(frac + EPS, 0.0f, 1.0f);
  Pose p0 = poseAtFraction(m, f0);
  Pose p1 = poseAtFraction(m, f1);
  return normalize3({p1.x - p0.x, p1.y - p0.y, p1.z - p0.z});
}

// Computes the geometry (line or arc) of a queued move, without starting it
// or touching cMove — used both for actually dispatching and for "peeking"
// at the next segment in the queue to compute the corner velocity. Returns
// false (with an error message) if the destination, or any intermediate
// point of the arc, isn't reachable.
bool planGeometry(const QueuedMove &m, const Pose &start, CartesianMoveState &out) {
  out.start = start;
  out.end = m.target;
  out.elbow = ELBOW_UP;

  if (m.type == CMD_G1) {
    out.type = PATH_LINE;
    float dx = m.target.x - start.x, dy = m.target.y - start.y, dz = m.target.z - start.z;
    out.totalLength = sqrtf(dx * dx + dy * dy + dz * dz);
    JointAngles endJoints;
    if (!solveAndCheckLimits(m.target, out.elbow, endJoints)) {
      return false;
    }
    return true;
  }

  // G2 / G3 (arc)
  out.type = PATH_ARC;
  Pose center = { start.x + m.offsetI, start.y + m.offsetJ, start.z };
  float rStart = hypotf(start.x - center.x, start.y - center.y);
  float rEnd   = hypotf(m.target.x - center.x, m.target.y - center.y);

  constexpr float ARC_TOLERANCE_MM = 0.5f;
  if (fabsf(rStart - rEnd) > ARC_TOLERANCE_MM) {
    Serial.println("ERROR: X/Y de destino no está a la misma distancia del centro (I/J) que el origen");
    return false;
  }
  float radius = (rStart + rEnd) * 0.5f;
  if (radius < 0.5f) {
    Serial.println("ERROR: radio de arco demasiado pequeño (revisa I/J)");
    return false;
  }

  float startAngle = atan2f(start.y - center.y, start.x - center.x);
  float endAngle   = atan2f(m.target.y - center.y, m.target.x - center.x);
  float delta = endAngle - startAngle;
  bool fullCircle = (fabsf(m.target.x - start.x) < 0.01f && fabsf(m.target.y - start.y) < 0.01f);
  bool clockwise = (m.type == CMD_G2);

  if (clockwise) {
    while (delta > 0.0f) delta -= 2.0f * PI;
    if (fullCircle) delta = -2.0f * PI;
  } else {
    while (delta < 0.0f) delta += 2.0f * PI;
    if (fullCircle) delta = 2.0f * PI;
  }

  out.center = center;
  out.radius = radius;
  out.startAngle = startAngle;
  out.deltaAngle = delta;
  out.totalLength = radius * fabsf(delta);

  if (out.totalLength < 0.05f) return true; // will be treated as a null move

  constexpr int ARC_CHECK_SAMPLES = 12;
  for (int i = 0; i <= ARC_CHECK_SAMPLES; i++) {
    float frac = (float)i / ARC_CHECK_SAMPLES;
    Pose p = poseAtFraction(out, frac);
    JointAngles sampleJoints;
    if (!solveAndCheckLimits(p, out.elbow, sampleJoints)) {
      return false;
    }
  }
  return true;
}

// Maximum velocity allowed at the junction between two segments, based on
// the angle between their directions of travel (0 = straight path, no extra
// limit; 180° = full reversal, must stop). "Junction deviation"-style
// formula (GRBL/Marlin): higher JUNCTION_DEVIATION_MM = faster tight
// corners but more real deviation from the ideal vertex.
float computeJunctionVelocity(Vec3 exitDir, Vec3 entryDir, float vmaxA, float vmaxB, float amax) {
  float cosTheta = -dot3(exitDir, entryDir);
  cosTheta = constrain(cosTheta, -1.0f, 1.0f);

  if (cosTheta < -0.9999f) return min(vmaxA, vmaxB); // practically straight
  if (cosTheta > 0.9999f) return 0.0f;                // practically a reversal

  float sinHalf = sqrtf(max(0.0f, (1.0f - cosTheta) * 0.5f));
  float denom = 1.0f - sinHalf;
  if (denom < 1e-4f) return 0.0f;

  float vJunction = sqrtf((amax * JUNCTION_DEVIATION_MM * sinHalf) / denom);
  return min(vJunction, min(vmaxA, vmaxB));
}

// Looks at the next segment in the queue (without popping or starting it)
// to know what velocity the segment about to be dispatched should end at.
float peekJunctionVelocity(const CartesianMoveState &current, float thisVmax) {
  QueuedMove next;
  if (!queuePeek(0, next)) return 0.0f;       // nothing after: must brake to a stop
  if (next.type == CMD_G0) return 0.0f;       // G0 always starts from zero velocity

  CartesianMoveState probe;
  if (!planGeometry(next, current.end, probe)) return 0.0f;
  if (probe.totalLength < 0.05f) return 0.0f; // the next one is a non-move

  float nextVmax = feedRateOrDefault(next.feedRate);
  Vec3 exitDir = pathTangentAtFraction(current, 1.0f);
  Vec3 entryDir = pathTangentAtFraction(probe, 0.0f);
  return computeJunctionVelocity(exitDir, entryDir, thisVmax, nextVmax, LINEAR_AMAX_MMPS2);
}

void beginCartesianMove(const Pose &target) {
  cMove.moveStartMicros = micros();
  cMove.lastTickMicros = cMove.moveStartMicros;
  cMove.warnedSaturation = false;

  cMove.lastCmdStepsJ1 = axisJ1.getPositionSteps();
  cMove.lastCmdStepsJ2 = axisJ2.getPositionSteps();
  cMove.lastCmdStepsZ  = axisZ.getPositionSteps();

  axisJ1.beginVelocityTracking();
  axisJ2.beginVelocityTracking();
  axisZ.beginVelocityTracking();

  cMove.active = true;
  currentTargetPose = target;
}

// Actually dispatches a G1/G2/G3 already popped from the queue: computes
// geometry, decides v0 (velocity inherited from the previous segment) and
// v1 (corner velocity with the next segment, if any), configures the
// profile, and starts.
bool dispatchCartesian(const QueuedMove &m) {
  Pose start = currentTargetPose;
  CartesianMoveState planned;
  if (!planGeometry(m, start, planned)) return false;

  if (planned.totalLength < 0.05f) {
    currentTargetPose = m.target;
    carryVelocity = 0.0f;
    return true; // negligible displacement, not worth interpolating
  }

  float vmax = feedRateOrDefault(m.feedRate);
  float v0 = min(carryVelocity, vmax);
  float v1 = peekJunctionVelocity(planned, vmax);

  planned.pathProfile.configure(planned.totalLength, v0, v1, vmax, LINEAR_AMAX_MMPS2);

  cMove = planned;
  beginCartesianMove(m.target);
  return true;
}

bool clampVelocity(float &v, float vmax) {
  if (v > vmax) { v = vmax; return true; }
  if (v < -vmax) { v = -vmax; return true; }
  return false;
}

// Advances the active cartesian segment by one interpolation tick.
void cartesianTick(float dt) {
  unsigned long now = micros();
  float elapsed = (now - cMove.moveStartMicros) / 1e6f;
  bool finished = cMove.pathProfile.isFinished(elapsed);
  float s = finished ? cMove.totalLength : cMove.pathProfile.positionAt(elapsed);
  float frac = s / cMove.totalLength;

  Pose p = poseAtFraction(cMove, frac);
  JointAngles j = kinematics.inverse(p, cMove.elbow);
  if (!j.reachable || !jointsWithinSoftLimits(j)) {
    Serial.println("ERROR: punto intermedio fuera de rango/límites, deteniendo movimiento");
    axisJ1.stopTracking(); axisJ2.stopTracking(); axisZ.stopTracking();
    cMove.active = false;
    carryVelocity = 0.0f;
    return;
  }

  long targetStepsJ1 = lroundf(j.theta1 * STEPS_PER_RAD_J1);
  long targetStepsJ2 = lroundf(j.theta2 * STEPS_PER_RAD_J2);
  long targetStepsZ  = lroundf(j.z * STEPS_PER_MM_Z);

  float vJ1 = (targetStepsJ1 - cMove.lastCmdStepsJ1) / dt;
  float vJ2 = (targetStepsJ2 - cMove.lastCmdStepsJ2) / dt;
  float vZ  = (targetStepsZ  - cMove.lastCmdStepsZ)  / dt;

  bool sat = clampVelocity(vJ1, J1_VMAX_SPS);
  sat |= clampVelocity(vJ2, J2_VMAX_SPS);
  sat |= clampVelocity(vZ, Z_VMAX_SPS);
  if (sat && !cMove.warnedSaturation) {
    Serial.println("AVISO: un eje llegó a su velocidad máxima durante el movimiento (posible cercanía a singularidad)");
    cMove.warnedSaturation = true;
  }

  axisJ1.setTrackingVelocity(vJ1);
  axisJ2.setTrackingVelocity(vJ2);
  axisZ.setTrackingVelocity(vZ);

  cMove.lastCmdStepsJ1 = targetStepsJ1;
  cMove.lastCmdStepsJ2 = targetStepsJ2;
  cMove.lastCmdStepsZ  = targetStepsZ;

  if (finished) {
    carryVelocity = cMove.pathProfile.getEndVelocity();
    axisJ1.stopTracking();
    axisJ2.stopTracking();
    axisZ.stopTracking();
    cMove.active = false;
  }
}

void updateCartesianInterpolation() {
  if (!cMove.active) return;
  unsigned long now = micros();
  if (now - cMove.lastTickMicros < INTERP_PERIOD_US) return;
  float dt = (now - cMove.lastTickMicros) / 1e6f;
  cMove.lastTickMicros = now;
  cartesianTick(dt);
}

bool isBusy() {
  return axisJ1.isMoving() || axisJ2.isMoving() || axisZ.isMoving() || cMove.active;
}

// If nothing is running and the queue has something, dispatches it. Called
// on every loop() pass, so a segment finishing and the next one starting
// happen within microseconds (no need to wait for the next Serial line).
void tryDispatchNext() {
  if (isBusy()) return;

  QueuedMove m;
  if (!queuePop(m)) return;

  bool ok;
  if (m.type == CMD_G0) {
    carryVelocity = 0.0f;
    ok = moveJointSyncTo(m.target);
  } else {
    ok = dispatchCartesian(m);
  }

  if (!ok) {
    // The segment failed (unreachable destination, inconsistent I/J, etc.);
    // the reason was already printed. We continue with the next one so a
    // single bad command doesn't block the whole queue.
    carryVelocity = 0.0f;
    tryDispatchNext();
  }
}

// ---------------- Cartesian position: read and set ----------------

// REAL cartesian position, computed via forward kinematics from each
// axis's current step count (not from currentTargetPose, which is the last
// *commanded* destination and may not match while a move is in progress).
Pose getCurrentPose() {
  JointAngles j;
  j.theta1 = axisJ1.getPositionSteps() / STEPS_PER_RAD_J1;
  j.theta2 = axisJ2.getPositionSteps() / STEPS_PER_RAD_J2;
  j.z = axisZ.getPositionSteps() / STEPS_PER_MM_Z;
  return kinematics.forward(j);
}

// Redefines the controller's CURRENT cartesian position without moving any
// motor — it's the cartesian version of HOME (which just zeroes the steps)
// or of what CALIBRATE does with each switch. Useful, for example, after
// repositioning the arm by hand, or to apply a fine calibration offset.
// Intentionally does NOT validate against software limits (same as HOME
// and CALIBRATE): this function describes where the arm really is, which
// could be beyond the normal operating range.
bool setCurrentPose(const Pose &p, ElbowConfig elbow = ELBOW_UP) {
  if (isBusy() || queueCount > 0) {
    Serial.println("ERROR: no se puede redefinir la posición con movimientos en marcha o en cola (probá CLEAR primero)");
    return false;
  }

  JointAngles j = kinematics.inverse(p, elbow);
  if (!j.reachable) {
    Serial.println("ERROR: esa posición no es alcanzable por la geometría del brazo");
    return false;
  }

  axisJ1.setPositionSteps(lroundf(j.theta1 * STEPS_PER_RAD_J1));
  axisJ2.setPositionSteps(lroundf(j.theta2 * STEPS_PER_RAD_J2));
  axisZ.setPositionSteps(lroundf(j.z * STEPS_PER_MM_Z));

  currentTargetPose = p;
  carryVelocity = 0.0f;
  return true;
}

void reportPosition() {
  Pose p = getCurrentPose();

  Serial.print("X="); Serial.print(p.x, 2);
  Serial.print(" Y="); Serial.print(p.y, 2);
  Serial.print(" Z="); Serial.print(p.z, 2);
  Serial.print("  | pasos J1="); Serial.print(axisJ1.getPositionSteps());
  Serial.print(" J2="); Serial.print(axisJ2.getPositionSteps());
  Serial.print(" Z="); Serial.print(axisZ.getPositionSteps());
  Serial.print("  | ocupado="); Serial.print(isBusy() ? "si" : "no");
  Serial.print(" cola="); Serial.println(queueCount);
}

// ---------------- Calibration (homing with switches) ----------------
// Two-stage routine, standard in CNC/3D-printer firmware:
//   1) Fast approach until the switch triggers.
//   2) Backing off until it releases + a bit more, then a SLOW re-approach —
//      that way the trigger point actually used for calibration is much
//      more repeatable (less affected by the inertia/speed of the first touch).
// It's blocking (doesn't process other commands while it runs): the step
// ISR keeps working regardless because it's a hardware interrupt, so
// motion isn't interrupted even though loop() is waiting inside a while.
bool waitForSwitch(uint8_t pin, bool wantTriggered, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while ((digitalRead(pin) == LOW) != wantTriggered) {
    if (millis() - t0 > timeoutMs) return false;
    yield(); // feeds the ESP32 watchdog during the blocking wait
  }
  return true;
}

// Moves the axis at a constant (signed) velocity 'sps' until it has covered
// 'steps' steps of distance, using the axis's own position counter (more
// reliable than estimating from elapsed time). Blocking.
void moveStepsBlocking(StepperAxis &axis, float signedSps, long steps) {
  long startPos = axis.getPositionSteps();
  axis.beginVelocityTracking();
  axis.setTrackingVelocity(signedSps);
  while (labs(axis.getPositionSteps() - startPos) < steps) {
    yield();
  }
  axis.setTrackingVelocity(0);
  axis.stopTracking();
}

bool homeAxis(StepperAxis &axis, uint8_t limitPin, int dirSign,
              float fastSps, float slowSps, float backoffSps,
              long knownPositionSteps, const char *axisName) {
  Serial.print("Calibrando "); Serial.print(axisName); Serial.println("...");

  // If we're already sitting on the switch at startup, clear it first.
  if (digitalRead(limitPin) == LOW) {
    axis.beginVelocityTracking();
    axis.setTrackingVelocity(-dirSign * backoffSps);
    bool ok = waitForSwitch(limitPin, false, HOMING_TIMEOUT_MS);
    axis.setTrackingVelocity(0);
    axis.stopTracking();
    if (!ok) {
      Serial.println("ERROR: no se pudo despejar el switch inicial (timeout)");
      return false;
    }
  }

  // 1) Fast approach
  axis.beginVelocityTracking();
  axis.setTrackingVelocity(dirSign * fastSps);
  bool hit = waitForSwitch(limitPin, true, HOMING_TIMEOUT_MS);
  axis.setTrackingVelocity(0);
  axis.stopTracking();
  if (!hit) {
    Serial.println("ERROR: timeout buscando el switch (revisá cableado/dirección/HOMING_DIR)");
    return false;
  }

  // 2) Back off until the switch releases, plus a bit of extra travel
  axis.beginVelocityTracking();
  axis.setTrackingVelocity(-dirSign * backoffSps);
  bool released = waitForSwitch(limitPin, false, HOMING_TIMEOUT_MS);
  axis.setTrackingVelocity(0);
  axis.stopTracking();
  if (!released) {
    Serial.println("ERROR: timeout retrocediendo del switch");
    return false;
  }
  moveStepsBlocking(axis, -dirSign * backoffSps, HOMING_BACKOFF_EXTRA_STEPS);

  // 3) Slow re-approach — this second touch is the one used to calibrate
  axis.beginVelocityTracking();
  axis.setTrackingVelocity(dirSign * slowSps);
  hit = waitForSwitch(limitPin, true, HOMING_TIMEOUT_MS);
  axis.setTrackingVelocity(0);
  axis.stopTracking();
  if (!hit) {
    Serial.println("ERROR: timeout en el reacercamiento lento");
    return false;
  }

  axis.setPositionSteps(knownPositionSteps); // known position at this exact point

  // 4) Move a bit away from the switch so it isn't left pressed
  moveStepsBlocking(axis, -dirSign * backoffSps, HOMING_CLEARANCE_STEPS);

  Serial.print(axisName); Serial.println(": calibrado.");
  return true;
}

bool runCalibration() {
  if (isBusy() || queueCount > 0) {
    Serial.println("ERROR: no se puede calibrar con movimientos en marcha o en cola (probá CLEAR primero)");
    return false;
  }

  Serial.println("=== Iniciando calibración (homing) ===");

  long j1Steps = lroundf(radians(J1_HOMING_ANGLE_DEG) * STEPS_PER_RAD_J1);
  long j2Steps = lroundf(radians(J2_HOMING_ANGLE_DEG) * STEPS_PER_RAD_J2);
  long zSteps  = lroundf(Z_HOMING_MM * STEPS_PER_MM_Z);

  bool ok = homeAxis(axisJ1, J1_LIMIT_PIN, HOMING_DIR_J1,
                      J1_VMAX_SPS * HOMING_FAST_FRACTION, J1_VMAX_SPS * HOMING_SLOW_FRACTION,
                      J1_VMAX_SPS * HOMING_BACKOFF_FRACTION, j1Steps, "J1")
         && homeAxis(axisJ2, J2_LIMIT_PIN, HOMING_DIR_J2,
                      J2_VMAX_SPS * HOMING_FAST_FRACTION, J2_VMAX_SPS * HOMING_SLOW_FRACTION,
                      J2_VMAX_SPS * HOMING_BACKOFF_FRACTION, j2Steps, "J2")
         && homeAxis(axisZ, Z_LIMIT_PIN, HOMING_DIR_Z,
                      Z_VMAX_SPS * HOMING_FAST_FRACTION, Z_VMAX_SPS * HOMING_SLOW_FRACTION,
                      Z_VMAX_SPS * HOMING_BACKOFF_FRACTION, zSteps, "Z");

  if (!ok) {
    Serial.println("=== Calibración ABORTADA ===");
    return false;
  }

  // The logical cartesian reference must reflect the real, just-calibrated
  // position, not whatever was assumed before homing.
  currentTargetPose = getCurrentPose();
  carryVelocity = 0.0f;

  Serial.println("=== Calibración completa ===");
  reportPosition();
  return true;
}

// ---------------- Very simple command parser ----------------
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "?") {
    reportPosition();
    return;
  }

  if (line == "HOME") {
    axisJ1.setPositionSteps(0);
    axisJ2.setPositionSteps(0);
    axisZ.setPositionSteps(0);
    Serial.println("Origen redefinido en la posición actual.");
    return;
  }

  if (line == "CLEAR") {
    queueClear();
    Serial.println("Cola vaciada (el tramo en marcha, si lo hay, sigue hasta terminar).");
    return;
  }

  if (line == "CALIBRATE") {
    runCalibration();
    return;
  }

  if (line.startsWith("SETPOS")) {
    Pose p = getCurrentPose(); // defaults to keeping whatever isn't specified
    int idx;
    if ((idx = line.indexOf('X')) >= 0) p.x = line.substring(idx + 1).toFloat();
    if ((idx = line.indexOf('Y')) >= 0) p.y = line.substring(idx + 1).toFloat();
    if ((idx = line.indexOf('Z')) >= 0) p.z = line.substring(idx + 1).toFloat();

    if (setCurrentPose(p)) {
      Serial.println("Posición redefinida (sin mover motores).");
      reportPosition();
    }
    return;
  }

  if (line.startsWith("G0") || line.startsWith("G1") || line.startsWith("G2") || line.startsWith("G3")) {
    QueuedMove m;
    m.type = line.startsWith("G0") ? CMD_G0 : line.startsWith("G1") ? CMD_G1 :
             line.startsWith("G2") ? CMD_G2 : CMD_G3;
    m.target = currentTargetPose; // default for anything not specified
    // NOTE: since commands are queued, "currentTargetPose" here reflects the
    // destination of the last QUEUED command (not necessarily the one
    // currently running), so unmentioned axes chain correctly across
    // consecutive commands.

    int idx;
    if ((idx = line.indexOf('X')) >= 0) m.target.x = line.substring(idx + 1).toFloat();
    if ((idx = line.indexOf('Y')) >= 0) m.target.y = line.substring(idx + 1).toFloat();
    if ((idx = line.indexOf('Z')) >= 0) m.target.z = line.substring(idx + 1).toFloat();
    if ((idx = line.indexOf('F')) >= 0) m.feedRate = line.substring(idx + 1).toFloat();
    if ((idx = line.indexOf('I')) >= 0) m.offsetI = line.substring(idx + 1).toFloat();
    if ((idx = line.indexOf('J')) >= 0) m.offsetJ = line.substring(idx + 1).toFloat();

    if (!queuePush(m)) {
      Serial.println("ERROR: cola llena, espera a que avance o manda menos comandos seguidos");
      return;
    }
    currentTargetPose = m.target; // so the next queued command chains correctly
    return;
  }

  Serial.println("Comando no reconocido. Usa: G0 | G1 | G2 | G3 X.. Y.. Z.. F.. I.. J.. | CALIBRATE | SETPOS X.. Y.. Z.. | CLEAR | HOME | ?");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  axisJ1.begin(J1_STEP_PIN, J1_DIR_PIN);
  axisJ2.begin(J2_STEP_PIN, J2_DIR_PIN);
  axisZ.begin(Z_STEP_PIN, Z_DIR_PIN);

  pinMode(J1_LIMIT_PIN, INPUT_PULLUP);
  pinMode(J2_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP);

  // Assumed initial position: arm fully extended along X (theta1=0, theta2=0)
  JointAngles home = {0.0f, 0.0f, 0.0f, true};
  currentTargetPose = kinematics.forward(home);

  // COMPATIBILITY NOTE: the timer API changed between versions of the ESP32
  // Arduino core. This code uses the "classic" API (core < 3.0, based on
  // esp-idf 4.x). If your IDE has core 3.x installed (esp-idf 5.x),
  // uncomment the block below and comment this one out.
  isrTimer = timerBegin(0, 80, true); // prescaler 80 -> 1 tick = 1us (with an 80MHz clock)
  timerAttachInterrupt(isrTimer, &onTimerISR, true);
  timerAlarmWrite(isrTimer, 1000000UL / ISR_FREQ_HZ, true); // period in us
  timerAlarmEnable(isrTimer);

  // --- New API (ESP32 core 3.x / esp-idf 5.x), use instead of the one above: ---
  // isrTimer = timerBegin(ISR_FREQ_HZ);       // frequency is passed directly now
  // timerAttachInterrupt(isrTimer, &onTimerISR);
  // timerAlarm(isrTimer, 1, true, 0);          // 1 tick = 1 timer period

  Serial.println("Controlador SCARA listo.");
  Serial.println("Comandos: G0 | G1 | G2 | G3 X.. Y.. Z.. F.. I.. J.. | CALIBRATE | SETPOS X.. Y.. Z.. | CLEAR | HOME | ?");
  Serial.println("Posición asumida al arrancar: theta1=0 theta2=0 z=0 (no calibrada). Corré CALIBRATE si tenés switches instalados.");
  reportPosition();
}

void loop() {
  updateCartesianInterpolation(); // advances the active cartesian segment, if any
  tryDispatchNext();              // starts the next one in the queue as soon as possible

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
}
