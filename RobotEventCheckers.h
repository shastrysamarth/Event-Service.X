#ifndef ROBOT_EVENT_CHECKERS_H
#define ROBOT_EVENT_CHECKERS_H

#include "BOARD.h"

void InitRobotEventCheckers(void);

uint8_t CheckRobotPeriodic(void);
uint8_t CheckBeaconEvents(void);
uint8_t CheckTapeEvents(void);
uint8_t CheckSolenoidEvents(void);
uint8_t CheckBumpEvents(void);
uint8_t CheckMisalignment(void);
uint8_t CheckDistanceMove(void);
uint8_t CheckAlignEvents(void);

#endif /* ROBOT_EVENT_CHECKERS_H */
