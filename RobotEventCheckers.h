#ifndef ROBOT_EVENT_CHECKERS_H
#define ROBOT_EVENT_CHECKERS_H

#include "BOARD.h"

void InitRobotEventCheckers(void);
void RobotEventCheckers_ToggleBeaconStream(void);
uint8_t RobotEventCheckers_GetRobotQueueMaxDepth(void);
uint16_t RobotEventCheckers_GetRobotQueuePostFailures(void);

uint8_t CheckRobotPeriodic(void);
uint8_t CheckBeaconEvents(void);
uint8_t CheckTapeEvents(void);
uint8_t CheckSolenoidEvents(void);
uint8_t CheckBumpEvents(void);
uint8_t CheckMisalignment(void);
uint8_t CheckDistanceMove(void);
uint8_t CheckAlignEvents(void);

#endif /* ROBOT_EVENT_CHECKERS_H */
