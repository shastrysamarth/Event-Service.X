#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include "BOARD.h"
#include "RobotTypes.h"

uint8_t RobotMotion_Init(void);

void RobotMotion_Stop(void);
void RobotMotion_Forward(float speedIPS);
void RobotMotion_Reverse(float speedIPS);
void RobotMotion_StrafeRight(float speedIPS);
void RobotMotion_StrafeLeft(float speedIPS);
void RobotMotion_TurnLeftAbout(TurnPivot_t pivot, float speedIPS);
void RobotMotion_TurnRightAbout(TurnPivot_t pivot, float speedIPS);
void RobotMotion_TestWheelSpeeds(float frontLeftIPS, float frontRightIPS,
        float rearLeftIPS, float rearRightIPS);

void RobotMotion_StartDistanceMove(DistanceAxis_t axis, int8_t direction, float targetInches);
void RobotMotion_StopDistanceMove(void);
uint8_t RobotMotion_IsDistanceMoveActive(void);
/* Completes after open-loop time = targetInches / MOTOR_SPEED_IPS (ES_Timer_GetTime). */
uint8_t RobotMotion_IsDistanceMoveComplete(void);
DistanceAxis_t RobotMotion_GetDistanceAxis(void);
int8_t RobotMotion_GetDistanceDirection(void);
/* DEPRECATED name: target inches; used only for debug display (completion is timed). */
float RobotMotion_GetDistanceTargetInches(void);

#endif /* ROBOT_MOTION_H */
