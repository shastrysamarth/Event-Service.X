#ifndef ROBOT_TEST_HARNESS_H
#define ROBOT_TEST_HARNESS_H

#include "BOARD.h"

uint8_t RobotTestHarness_CheckKeyboard(void);
void RobotTestHarness_PrintHelp(void);
void RobotTestHarness_RunMotorSensorBench(void);
void RobotTestHarness_RunBeaconBench(void);
void RobotTestHarness_RunGPIOHighBench(void);

#endif /* ROBOT_TEST_HARNESS_H */
