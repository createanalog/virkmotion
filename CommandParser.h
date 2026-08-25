#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

// ============================================================================
// CommandParser.h — parses one line of Serial input into an action: report
// status (?), redefine the origin (HOME/SETPOS), clear the queue (CLEAR),
// run homing (CALIBRATE), or push a motion command (G0/G1/G2/G3) onto the
// move queue. Also owns reportPosition(), the "?" status report.
// ============================================================================

#include <Arduino.h>
#include "Kinematics.h"
#include "Types.h"
#include "StepperAxis.h"
#include "CommandQueue.h"
#include "Homing.h"

// Defined in virkmotion.ino.
extern bool isBusy();
extern StepperAxis axisJ1;
extern StepperAxis axisJ2;
extern StepperAxis axisZ;
extern Pose getCurrentPose();
extern bool setCurrentPose(const Pose &p, ElbowConfig elbow = ELBOW_UP);
extern Pose currentTargetPose;

void reportPosition();

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
    Serial.println("Origin redefined at the current position.");
    return;
  }

  if (line == "CLEAR") {
    queueClear();
    Serial.println("Queue cleared (the segment already in progress, if any, will still finish).");
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
      Serial.println("Position redefined (no motors moved).");
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
      Serial.println("ERROR: queue full, wait for it to advance or send fewer commands at once");
      return;
    }
    currentTargetPose = m.target; // so the next queued command chains correctly
    return;
  }

  Serial.println("Unrecognized command. Use: G0 | G1 | G2 | G3 X.. Y.. Z.. F.. I.. J.. | CALIBRATE | SETPOS X.. Y.. Z.. | CLEAR | HOME | ?");
}

void reportPosition() {
  Pose p = getCurrentPose();

  Serial.print("X="); Serial.print(p.x, 2);
  Serial.print(" Y="); Serial.print(p.y, 2);
  Serial.print(" Z="); Serial.print(p.z, 2);
  Serial.print("  | steps J1="); Serial.print(axisJ1.getPositionSteps());
  Serial.print(" J2="); Serial.print(axisJ2.getPositionSteps());
  Serial.print(" Z="); Serial.print(axisZ.getPositionSteps());
  Serial.print("  | busy="); Serial.print(isBusy() ? "yes" : "no");
  Serial.print(" queue="); Serial.println(queueCount);
}

#endif
