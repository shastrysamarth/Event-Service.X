#ifndef ROBOT_LAUNCHER_H
#define ROBOT_LAUNCHER_H

#include "BOARD.h"

uint8_t RobotLauncher_Init(void);
void RobotLauncher_SetAngleFromBeaconADC(uint16_t beaconADC);
void RobotLauncher_StartShooter(void);
void RobotLauncher_StopShooter(void);

#endif /* ROBOT_LAUNCHER_H */
