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
#include "Types.h"
#include "RobotConfig.h"
#include "Homing.h"
#include "CommandParser.h"
#include "CommandQueue.h"
#include "CartesianInterpolation.h"

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
bool setCurrentPose(const Pose &p, ElbowConfig elbow) {
  if (isBusy() || queueCount > 0) {
    Serial.println("ERROR: cannot redefine the position while a move is in progress or queued (try CLEAR first)");
    return false;
  }

  JointAngles j = kinematics.inverse(p, elbow);
  if (!j.reachable) {
    Serial.println("ERROR: that position is not reachable by the arm's geometry");
    return false;
  }

  axisJ1.setPositionSteps(lroundf(j.theta1 * STEPS_PER_RAD_J1));
  axisJ2.setPositionSteps(lroundf(j.theta2 * STEPS_PER_RAD_J2));
  axisZ.setPositionSteps(lroundf(j.z * STEPS_PER_MM_Z));

  currentTargetPose = p;
  carryVelocity = 0.0f;
  return true;
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
  // Arduino core. This code uses the NEW API (core 3.x, based on esp-idf 5.x).
  // If your IDE has an older core (< 3.0, esp-idf 4.x) installed, comment
  // this block out and uncomment the "classic API" block below instead.
  isrTimer = timerBegin(1000000); // tick = 1us
  timerAttachInterrupt(isrTimer, &onTimerISR);
  timerAlarm(isrTimer, 1000000UL / ISR_FREQ_HZ, true, 0); // period in us

  // --- Classic API (ESP32 core < 3.0 / esp-idf 4.x), use instead of the block above: ---
  // isrTimer = timerBegin(0, 80, true);              // prescaler 80 -> 1 tick = 1us (with an 80MHz clock)
  // timerAttachInterrupt(isrTimer, &onTimerISR, true);
  // timerAlarmWrite(isrTimer, 1000000UL / ISR_FREQ_HZ, true); // period in us
  // timerAlarmEnable(isrTimer);

  Serial.println("SCARA controller ready.");
  Serial.println("Commands: G0 | G1 | G2 | G3 X.. Y.. Z.. F.. I.. J.. | CALIBRATE | SETPOS X.. Y.. Z.. | CLEAR | HOME | ?");
  Serial.println("Assumed startup position: theta1=0 theta2=0 z=0 (not calibrated). Run CALIBRATE if limit switches are installed.");
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
