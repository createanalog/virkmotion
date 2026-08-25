

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