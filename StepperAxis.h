#ifndef STEPPER_AXIS_H
#define STEPPER_AXIS_H

#include <Arduino.h>
#include "TrapProfile.h"

// Represents an axis driven by a step/dir stepper driver, such as a
// TB6560 or similar. Generates pulses through a phase accumulator (NCO)
// updated at a fixed rate from a timer ISR, which allows the velocity to
// vary continuously without recomputing the interrupt interval for every
// single step.
//
// Has two modes of use:
//  - TRAP_MOVE: the axis itself computes and follows its own internal
//    trapezoidal profile (used by moveTo(), joint-space synchronized motion).
//  - VELOCITY_TRACK: an external controller (the linear interpolator) feeds
//    it the instantaneous velocity it should run at, tick by tick. The axis
//    just generates pulses at that velocity until told otherwise.
class StepperAxis {
  public:
    void begin(uint8_t stepPin, uint8_t dirPin, bool invertDir = false) {
      this->stepPin = stepPin;
      this->dirPin = dirPin;
      this->invertDir = invertDir;
      pinMode(stepPin, OUTPUT);
      pinMode(dirPin, OUTPUT);
      digitalWrite(stepPin, LOW);
      digitalWrite(dirPin, LOW);
    }

    // ---------------- TRAP_MOVE mode (synchronized joint-space motion) ----------------

    // Prepares the move but allows rescaling its duration before starting
    // (used by the multi-axis planner to synchronize arrival).
    void prepareMove(long deltaSteps, float vmaxStepsPerSec, float amaxStepsPerSec2) {
      pendingDelta = deltaSteps;
      float totalSteps = fabsf((float)deltaSteps);
      profile.configure(totalSteps, vmaxStepsPerSec, amaxStepsPerSec2);
    }

    float getNaturalTime() const {
      return profile.getTotalTime();
    }

    void rescaleAndStart(float targetTime) {
      profile.rescaleToTime(targetTime);
      setDirection(pendingDelta >= 0);
      elapsed = 0.0f;
      phase = 0.0f;
      stepsRemaining = labs(pendingDelta);
      mode = MODE_TRAP_MOVE;
      moving = stepsRemaining > 0;
      stepPinState = false;
    }

    // ---------------- VELOCITY_TRACK mode (linear interpolation) ----------------

    // Enables velocity-tracking mode. Call before the first call to
    // setTrackingVelocity() for a linear move.
    void beginVelocityTracking() {
      portENTER_CRITICAL(&axisMux);
      mode = MODE_VELOCITY_TRACK;
      commandedVelocity = 0.0f;
      phase = 0.0f;
      moving = true;
      portEXIT_CRITICAL(&axisMux);
    }

    // Sets the instantaneous (signed) velocity in steps/s. Called from the
    // main loop (not the ISR) at the interpolation frequency.
    void setTrackingVelocity(float stepsPerSec) {
      portENTER_CRITICAL(&axisMux);
      setDirection(stepsPerSec >= 0.0f);
      commandedVelocity = fabsf(stepsPerSec);
      portEXIT_CRITICAL(&axisMux);
    }

    void stopTracking() {
      portENTER_CRITICAL(&axisMux);
      commandedVelocity = 0.0f;
      moving = false;
      mode = MODE_IDLE;
      portEXIT_CRITICAL(&axisMux);
    }

    // ---------------- Common ----------------

    // Must be called from the timer ISR at a fixed interval dt (seconds).
    // Generates at most one STEP edge (rising or falling) per call; that's
    // why dt must be small enough relative to 1/vmax (see ISR_FREQ_HZ).
    void IRAM_ATTR update(float dt) {
      if (stepPinState) {
        // Second half of the STEP pulse: bring it back down
        digitalWrite(stepPin, LOW);
        stepPinState = false;
        return;
      }

      if (!moving) return;

      float v;
      if (mode == MODE_TRAP_MOVE) {
        elapsed += dt;
        v = profile.velocityAt(elapsed);
      } else {
        v = commandedVelocity; // set externally, constant until the next tick
      }

      phase += v * dt;

      bool canStep = (mode == MODE_TRAP_MOVE) ? (stepsRemaining > 0) : true;
      if (phase >= 1.0f && canStep) {
        phase -= 1.0f;
        digitalWrite(stepPin, HIGH);
        stepPinState = true;
        if (mode == MODE_TRAP_MOVE) stepsRemaining--;
        currentPosition += direction ? 1 : -1;
      }

      if (mode == MODE_TRAP_MOVE && profile.isFinished(elapsed) && stepsRemaining == 0) {
        moving = false;
      }
    }

    bool isMoving() const { return moving; }
    long getPositionSteps() const { return currentPosition; }
    void setPositionSteps(long steps) { currentPosition = steps; }

  private:
    enum Mode { MODE_IDLE, MODE_TRAP_MOVE, MODE_VELOCITY_TRACK };

    uint8_t stepPin = 0;
    uint8_t dirPin = 0;
    bool invertDir = false;

    TrapProfile profile;
    volatile Mode mode = MODE_IDLE;
    volatile bool moving = false;
    volatile long stepsRemaining = 0;
    volatile long currentPosition = 0;
    long pendingDelta = 0;
    volatile bool direction = true;
    volatile float commandedVelocity = 0.0f;

    float elapsed = 0.0f;
    float phase = 0.0f;
    volatile bool stepPinState = false;

    portMUX_TYPE axisMux = portMUX_INITIALIZER_UNLOCKED;

    void setDirection(bool dirForward) {
      if (dirForward != direction) {
        direction = dirForward;
        digitalWrite(dirPin, direction != invertDir ? HIGH : LOW);
      }
    }
};

#endif
