#ifndef TRAP_PROFILE_H
#define TRAP_PROFILE_H

#include <Arduino.h>

// Velocity profile for a single segment, in "distance units" (steps, or mm
// if used for a cartesian path). Generalizes the classic trapezoid (starts
// and ends at rest) so it can start and/or end at a nonzero velocity —
// that way, when chaining moves (see the move queue in the .ino), there's
// no need to fully stop at every junction if the geometry allows it.
//
// Profile shape: accelerates from v0 up to a "peak" vPeak, optionally
// cruises at vPeak, and decelerates from vPeak down to v1. If the available
// distance isn't enough to reach the requested v1 (for example, a very
// short segment between two fast moves), the profile computes the best
// achievable v1 given the distance and amax — getEndVelocity() returns that
// REAL value, which may differ from the one requested.
class TrapProfile {
  public:
    // Backward-compatible with the previous usage: classic rest-to-rest profile.
    void configure(float totalDistance, float vmax, float amax) {
      configure(totalDistance, 0.0f, 0.0f, vmax, amax);
    }

    // General profile: starts at v0, tries to end at v1 (mm/s or steps/s,
    // matching distance's unit), never exceeding vmax nor accelerating/
    // decelerating faster than amax. v1 may not be reachable if distance is
    // too short; use getEndVelocity() to find out what v1 was actually achieved.
    void configure(float totalDistance, float v0In, float v1In, float vmaxIn, float amaxIn) {
      distance = max(0.0f, totalDistance);
      vmax = max(0.0f, vmaxIn);
      amax = max(0.0001f, amaxIn); // avoid division by zero
      v0 = constrain(v0In, 0.0f, vmax);
      v1Target = constrain(v1In, 0.0f, vmax);
      recompute();
    }

    // Stretches (or compresses) the profile in time so it lasts exactly
    // targetTime seconds, preserving distance and the v0/v1 velocities.
    // Only makes sense for rest-to-rest profiles (v0=v1=0), which is how
    // it's used in joint-space synchronized motion (G0).
    void rescaleToTime(float targetTime) {
      if (totalTime <= 0.0f || targetTime <= 0.0f) return;
      float k = targetTime / totalTime;
      vmax = vmax / k;
      amax = amax / (k * k);
      recompute();
    }

    float getTotalTime() const { return totalTime; }

    // REAL velocity reached at the end of the segment (can be less, or more
    // if you were decelerating and ran out of distance, than the requested v1).
    float getEndVelocity() const { return v1Achieved; }

    float velocityAt(float t) const {
      if (t <= 0.0f) return v0;
      if (t >= totalTime) return v1Achieved;
      if (t < accelTime) {
        return v0 + amax * t;
      } else if (t < accelTime + cruiseTime) {
        return vPeak;
      } else {
        float td = t - (accelTime + cruiseTime);
        return vPeak - amax * td;
      }
    }

    // Distance traveled at instant t. It's the integral of velocityAt();
    // used for path interpolation (line or arc).
    float positionAt(float t) const {
      if (distance <= 0.0f || t <= 0.0f) return 0.0f;
      if (t >= totalTime) return distance;

      if (t < accelTime) {
        return v0 * t + 0.5f * amax * t * t;
      }
      float accelDist = v0 * accelTime + 0.5f * amax * accelTime * accelTime;

      if (t < accelTime + cruiseTime) {
        return accelDist + vPeak * (t - accelTime);
      }
      float cruiseDist = vPeak * cruiseTime;
      float td = t - (accelTime + cruiseTime);
      return accelDist + cruiseDist + (vPeak * td - 0.5f * amax * td * td);
    }

    bool isFinished(float t) const {
      return totalTime <= 0.0f || t >= totalTime;
    }

  private:
    float distance = 0, vmax = 0, amax = 0.0001f;
    float v0 = 0, v1Target = 0, v1Achieved = 0;

    float vPeak = 0;
    float accelTime = 0, cruiseTime = 0, decelTime = 0, totalTime = 0;

    void recompute() {
      if (distance <= 0.0f) {
        // No distance to travel: there's no time to change velocity, so we
        // "arrive" already at whatever velocity we came in with.
        vPeak = v0;
        v1Achieved = v0;
        accelTime = cruiseTime = decelTime = totalTime = 0.0f;
        return;
      }

      // "Free" peak that would result from accelerating from v0 to vPeak
      // and then decelerating from vPeak to v1Target, using the full distance:
      //   (vPeak²-v0²)/(2a) + (vPeak²-v1²)/(2a) = distance
      float vPeakCandidate = sqrtf(max(0.0f, amax * distance + (v0 * v0 + v1Target * v1Target) * 0.5f));
      float vPeakClamped = min(vPeakCandidate, vmax);

      if (vPeakClamped < v0) {
        // Not even braking the whole segment reaches a peak above v0: the
        // distance only allows a partial deceleration from v0. No
        // acceleration phase at all.
        vPeak = v0;
        v1Achieved = sqrtf(max(0.0f, v0 * v0 - 2.0f * amax * distance));
        accelTime = 0.0f;
        decelTime = (vPeak - v1Achieved) / amax;
        cruiseTime = 0.0f;
      } else if (vPeakClamped < v1Target) {
        // Not even accelerating flat-out the whole segment reaches the
        // requested v1: pure acceleration, no deceleration phase.
        v1Achieved = sqrtf(v0 * v0 + 2.0f * amax * distance);
        vPeak = v1Achieved;
        accelTime = (vPeak - v0) / amax;
        decelTime = 0.0f;
        cruiseTime = 0.0f;
      } else {
        // Normal case: accelerate to vPeak, (maybe) cruise, decelerate to v1Target.
        vPeak = vPeakClamped;
        v1Achieved = v1Target;
        accelTime = (vPeak - v0) / amax;
        decelTime = (vPeak - v1Achieved) / amax;
        float accelDist = v0 * accelTime + 0.5f * amax * accelTime * accelTime;
        float decelDist = v1Achieved * decelTime + 0.5f * amax * decelTime * decelTime;
        float cruiseDist = max(0.0f, distance - accelDist - decelDist);
        cruiseTime = (vPeak > 0.0f) ? (cruiseDist / vPeak) : 0.0f;
      }

      totalTime = accelTime + cruiseTime + decelTime;
    }
};

#endif
