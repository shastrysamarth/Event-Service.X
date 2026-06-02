#ifndef ROBOT_SENSORS_H
#define ROBOT_SENSORS_H

#include "BOARD.h"
#include "RobotTypes.h"

uint8_t RobotSensors_Init(void);

uint16_t RobotSensors_ReadBeaconRawADC(void);
uint16_t RobotSensors_GetBeaconRawADC(void);
uint16_t RobotSensors_PushBeaconADC(uint16_t reading);
uint16_t RobotSensors_ReadBeaconADC(void);
uint16_t RobotSensors_GetBeaconADC(void);
uint8_t  RobotSensors_IsBeaconAverageReady(void);
void     RobotSensors_ResetBeaconAverage(void);
uint8_t  RobotSensors_ReadTapeDigital(TapeSensor_t sensor);
uint8_t  RobotSensors_GetTapeDigital(TapeSensor_t sensor);
uint16_t RobotSensors_ReadSolenoidADC(SolenoidSensor_t sensor);
uint8_t  RobotSensors_ReadBumpDigital(BumpSensor_t sensor);
uint8_t  RobotSensors_GetBumpDigital(BumpSensor_t sensor);
uint16_t RobotSensors_ReadShooterMotorADC(void);
uint8_t  RobotSensors_ReadBeaconDistanceFeet(void);
uint8_t  RobotSensors_BeaconDistanceFeetFromADC(uint16_t reading);

uint8_t RobotSensors_IsTapeOn(TapeSensor_t sensor);
uint8_t RobotSensors_TapeRawIsOn(TapeSensor_t sensor, uint8_t rawReading);
uint8_t RobotSensors_IsSolenoidOn(SolenoidSensor_t sensor);
uint8_t RobotSensors_IsBumpOn(BumpSensor_t sensor);
uint8_t RobotSensors_BumpRawIsOn(uint8_t rawReading);

#endif /* ROBOT_SENSORS_H */
