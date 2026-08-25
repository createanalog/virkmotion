#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

// End-effector pose: cartesian position (mm) + Z height (mm)
struct Pose {
  float x;
  float y;
  float z;
};

// Joint angles (radians) + Z linear position (mm)
struct JointAngles {
  float theta1;
  float theta2;
  float z;
  bool reachable; // false if the point is outside the workspace
};

enum ElbowConfig {
  ELBOW_UP,
  ELBOW_DOWN
};

class ScaraKinematics {
  public:
    // L1, L2: length of the arm's two links, in mm
    ScaraKinematics(float linkLength1, float linkLength2)
      : L1(linkLength1), L2(linkLength2) {}

    // Forward kinematics: from joint angles to cartesian position
    Pose forward(const JointAngles &joints) const {
      Pose p;
      p.x = L1 * cosf(joints.theta1) + L2 * cosf(joints.theta1 + joints.theta2);
      p.y = L1 * sinf(joints.theta1) + L2 * sinf(joints.theta1 + joints.theta2);
      p.z = joints.z;
      return p;
    }

    // Inverse kinematics: from cartesian position to joint angles.
    // elbow selects between the two possible solutions (elbow up/down).
    JointAngles inverse(const Pose &target, ElbowConfig elbow = ELBOW_UP) const {
      JointAngles result;
      result.z = target.z;

      float r2 = target.x * target.x + target.y * target.y;
      float r = sqrtf(r2);

      // Check that the point is within the arm's workspace
      if (r > (L1 + L2) || r < fabsf(L1 - L2)) {
        result.reachable = false;
        result.theta1 = 0;
        result.theta2 = 0;
        return result;
      }

      float cosTheta2 = (r2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
      cosTheta2 = constrain(cosTheta2, -1.0f, 1.0f); // numerical safety

      float theta2 = acosf(cosTheta2);
      if (elbow == ELBOW_DOWN) {
        theta2 = -theta2;
      }

      float k1 = L1 + L2 * cosf(theta2);
      float k2 = L2 * sinf(theta2);
      float theta1 = atan2f(target.y, target.x) - atan2f(k2, k1);

      result.theta1 = theta1;
      result.theta2 = theta2;
      result.reachable = true;
      return result;
    }

    float getL1() const { return L1; }
    float getL2() const { return L2; }

  private:
    float L1;
    float L2;
};

#endif
