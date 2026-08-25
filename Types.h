

struct Vec3 { float x, y, z; };

enum MoveCmdType { CMD_G0, CMD_G1, CMD_G2, CMD_G3 };

struct QueuedMove {
  MoveCmdType type;
  Pose target;
  float feedRate = -1.0f;   // <=0 means "use LINEAR_VMAX_MMPS"
  float offsetI = 0.0f, offsetJ = 0.0f; // G2/G3 only
};

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