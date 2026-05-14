#ifndef ROBOT_STEPPER_H
#define ROBOT_STEPPER_H

#include "BOARD.h"

uint8_t RobotStepper_Init(void);
void RobotStepper_Enable(void);
void RobotStepper_Disable(void);
void RobotStepper_SetDirection(uint8_t forward);
void RobotStepper_PulseStep(void);

#endif /* ROBOT_STEPPER_H */
