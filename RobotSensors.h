#ifndef ROBOT_SENSORS_H
#define ROBOT_SENSORS_H

#include "BOARD.h"
#include "RobotTypes.h"

uint8_t RobotSensors_Init(void);

uint16_t RobotSensors_ReadBeaconADC(void);
uint16_t RobotSensors_ReadTapeADC(TapeSensor_t sensor);
uint16_t RobotSensors_ReadSolenoidADC(SolenoidSensor_t sensor);
uint16_t RobotSensors_ReadBumpADC(BumpSensor_t sensor);
uint16_t RobotSensors_ReadShooterMotorADC(void);

uint8_t RobotSensors_IsTapeOn(TapeSensor_t sensor);
uint8_t RobotSensors_IsSolenoidOn(SolenoidSensor_t sensor);
uint8_t RobotSensors_IsBumpOn(BumpSensor_t sensor);

#endif /* ROBOT_SENSORS_H */
