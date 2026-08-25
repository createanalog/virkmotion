#ifndef CARTESIAN_INTERPOLATION_H
#define CARTESIAN_INTERPOLATION_H

// ============================================================================
// CartesianInterpolation.h — real cartesian moves (G1 straight lines, G2/G3
// arcs): path geometry, software-limit checking, corner ("junction")
// velocity between consecutive queued segments, and the per-tick
// interpolator that turns the active segment into a per-axis velocity
// command for StepperAxis's VELOCITY_TRACK mode.
// ============================================================================

#include <Arduino.h>
#include "Kinematics.h"
#include "Types.h"
#include "RobotConfig.h"
#include "StepperAxis.h"
#include "CommandQueue.h"

// Defined in virkmotion.ino.
extern ScaraKinematics kinematics;
extern StepperAxis axisJ1;
extern StepperAxis axisJ2;
extern StepperAxis axisZ;
extern Pose currentTargetPose;
extern float carryVelocity;

struct Vec3 { float x, y, z; };

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
    Serial.println("ERROR: point outside the workspace (arm geometry)");
    return false;
  }
  if (!jointsWithinSoftLimits(outJoints)) {
    Serial.println("ERROR: point outside the software limits (configured safe range)");
    return false;
  }
  return true;
}

// ---------------- Minimal 3D vectors (only what's needed for directions) ----------------

Vec3 normalize3(Vec3 v) {
  float len = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  if (len < 1e-6f) return {0, 0, 0};
  return {v.x / len, v.y / len, v.z / len};
}
float dot3(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// ---------------- G1/G2/G3: real cartesian interpolation ----------------

CartesianMoveState cMove;

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
  const float eps = 0.001f;
  float f0 = constrain(frac - eps, 0.0f, 1.0f);
  float f1 = constrain(frac + eps, 0.0f, 1.0f);
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
    Serial.println("ERROR: target X/Y is not the same distance from the center (I/J) as the start point");
    return false;
  }
  float radius = (rStart + rEnd) * 0.5f;
  if (radius < 0.5f) {
    Serial.println("ERROR: arc radius too small (check I/J)");
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
    Serial.println("ERROR: intermediate point out of range/limits, stopping the move");
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
    Serial.println("WARNING: an axis hit its maximum velocity during this move (possibly near a singularity)");
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

#endif
