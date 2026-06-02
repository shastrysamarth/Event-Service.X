#ifndef ROBOT_IMU_H
#define ROBOT_IMU_H

#include "BOARD.h"

uint8_t RobotIMU_Init(void);
uint8_t RobotIMU_BeginNDOF(void);
uint8_t RobotIMU_IsFullyCalibrated(void);

void RobotIMU_Update(void);
void RobotIMU_UpdateGyro(void);
void RobotIMU_ZeroAll(void);
void RobotIMU_ZeroHeading(void);
void RobotIMU_LatchReferenceHeading(void);
void RobotIMU_ZeroPositionVelocity(void);

float RobotIMU_GetHeadingDeg(void);
float RobotIMU_GetXInches(void);
float RobotIMU_GetYInches(void);
float RobotIMU_GetHeadingErrorToZeroDeg(void);
float RobotIMU_GetReferenceHeadingDeg(void);
float RobotIMU_GetHeadingErrorToRefDeg(void);
uint8_t RobotIMU_IsReferenceHeadingLatched(void);
uint8_t RobotIMU_IsReady(void);
float RobotIMU_GetHeadingOffsetDeg(void);
float RobotIMU_GetXVelocityIPS(void);
float RobotIMU_GetYVelocityIPS(void);
uint8_t RobotIMU_IsStationary(void);
float RobotIMU_GetXAccelIPS2(void);
float RobotIMU_GetYAccelIPS2(void);
float RobotIMU_GetZGyroDPS(void);
void RobotIMU_PrintDebugSnapshot(void);
/* Gyro-only snapshot (raw X/Y/Z deg/s, heading-axis rate, and integrated
 * angle). Caller refreshes the data via RobotIMU_Update() first. */
void RobotIMU_PrintGyroSnapshot(void);
/* Integrate the latest gyro rates over dtMs into the per-axis accumulated
 * angle (deg). Reset the running total with RobotIMU_ResetGyroAccum(). */
void RobotIMU_AccumulateGyro(uint32_t dtMs);
void RobotIMU_ResetGyroAccum(void);
void RobotIMU_SetDebugStream(uint8_t enabled);
void RobotIMU_ToggleDebugStream(void);
uint8_t RobotIMU_IsDebugStreamEnabled(void);
void RobotIMU_DebugStreamTick(void);
uint8_t RobotIMU_EnsureNDOF(void);

#endif /* ROBOT_IMU_H */
