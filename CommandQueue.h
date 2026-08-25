#ifndef COMMAND_QUEUE_H
#define COMMAND_QUEUE_H

// ============================================================================
// CommandQueue.h — fixed-size ring buffer of pending G0/G1/G2/G3 moves.
// CommandParser pushes into it as commands arrive over Serial; the main
// loop's dispatcher (tryDispatchNext, in virkmotion.ino) pops and executes
// them in order, one at a time, as soon as the previous one finishes.
// ============================================================================

#include "Types.h"
#include "RobotConfig.h"

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

#endif
