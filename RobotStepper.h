#ifndef ROBOT_STEPPER_H
#define ROBOT_STEPPER_H

#include "BOARD.h"

uint8_t RobotStepper_Init(void);
void RobotStepper_Enable(void);
void RobotStepper_Disable(void);
void RobotStepper_SetDirection(uint8_t forward);
void RobotStepper_PulseStep(void);
void RobotStepper_Step(uint16_t steps, uint8_t forward);
void RobotStepper_MoveToStep(int16_t targetStep);
uint8_t RobotStepper_StepTowardTarget(int16_t targetStep);
void RobotStepper_ZeroPosition(void);
int16_t RobotStepper_GetPosition(void);

#endif /* ROBOT_STEPPER_H */
