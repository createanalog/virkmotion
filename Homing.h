// ---------------- Calibration (homing with switches) ----------------
// Two-stage routine, standard in CNC/3D-printer firmware:
//   1) Fast approach until the switch triggers.
//   2) Backing off until it releases + a bit more, then a SLOW re-approach —
//      that way the trigger point actually used for calibration is much
//      more repeatable (less affected by the inertia/speed of the first touch).
// It's blocking (doesn't process other commands while it runs): the step
// ISR keeps working regardless because it's a hardware interrupt, so
// motion isn't interrupted even though loop() is waiting inside a while.

extern bool isBusy();
extern void reportPosition();
extern StepperAxis axisJ1;
extern StepperAxis axisJ2;
extern StepperAxis axisZ;
extern int queueCount;
extern Pose getCurrentPose();
extern Pose currentTargetPose;
extern float carryVelocity;

bool waitForSwitch(uint8_t pin, bool wantTriggered, unsigned long timeoutMs);
void moveStepsBlocking(StepperAxis &axis, float signedSps, long steps);
bool homeAxis(StepperAxis &axis, uint8_t limitPin, int dirSign,
              float fastSps, float slowSps, float backoffSps,
              long knownPositionSteps, const char *axisName);

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

// ------ HELPERS ------ 

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
