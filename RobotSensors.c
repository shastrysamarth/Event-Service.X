#include "RobotSensors.h"

#include "AD.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"

static unsigned int TapeADPin(TapeSensor_t sensor);
static unsigned int SolenoidADPin(SolenoidSensor_t sensor);
static unsigned int BumpADPin(BumpSensor_t sensor);
static uint8_t IsThresholdActive(uint16_t reading, uint16_t threshold, uint8_t activeHigh);

uint8_t RobotSensors_Init(void)
{
    unsigned int pins = 0u;

#if ROBOT_PLUGPLAY_USE_TAPE1_ADC
    pins |= TAPE_SENSOR_1_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_TAPE2_ADC
    pins |= TAPE_SENSOR_2_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_TAPE3_ADC
    pins |= TAPE_SENSOR_3_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_TAPE4_ADC
    pins |= TAPE_SENSOR_4_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_TAPE5_ADC
    pins |= TAPE_SENSOR_5_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_TAPE6_ADC
    pins |= TAPE_SENSOR_6_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_TAPE7_ADC
    pins |= TAPE_SENSOR_7_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_TAPE8_ADC
    pins |= TAPE_SENSOR_8_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_TAPE9_ADC
    pins |= TAPE_SENSOR_9_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    pins |= BEACON_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_SOLENOID1_ADC
    pins |= SOLENOID_SENSOR_1_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_SOLENOID2_ADC
    pins |= SOLENOID_SENSOR_2_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_SOLENOID3_ADC
    pins |= SOLENOID_SENSOR_3_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_SOLENOID4_ADC
    pins |= SOLENOID_SENSOR_4_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_SOLENOID5_ADC
    pins |= SOLENOID_SENSOR_5_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_SOLENOID6_ADC
    pins |= SOLENOID_SENSOR_6_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_BUMP1_ADC
    pins |= BUMP_SENSOR_1_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_BUMP2_ADC
    pins |= BUMP_SENSOR_2_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_BUMP3_ADC
    pins |= BUMP_SENSOR_3_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_BUMP4_ADC
    pins |= BUMP_SENSOR_4_ADC_PIN;
#endif
#if ROBOT_PLUGPLAY_USE_SHOOTER_ADC
    pins |= SHOOTER_MOTOR_ADC_PIN;
#endif

    if (pins == 0u) {
        return TRUE;
    }
    return (AD_AddPins(pins) == SUCCESS) ? TRUE : FALSE;
}

uint16_t RobotSensors_ReadBeaconADC(void)
{
#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    return AD_ReadADPin(BEACON_ADC_PIN);
#else
    return 0u;
#endif
}

uint16_t RobotSensors_ReadTapeADC(TapeSensor_t sensor)
{
    return RobotPlugPlay_IsTapeADCEnabled((uint8_t) sensor) ?
            AD_ReadADPin(TapeADPin(sensor)) : 0u;
}

uint16_t RobotSensors_ReadSolenoidADC(SolenoidSensor_t sensor)
{
    return RobotPlugPlay_IsSolenoidADCEnabled((uint8_t) sensor) ?
            AD_ReadADPin(SolenoidADPin(sensor)) : 0u;
}

uint16_t RobotSensors_ReadBumpADC(BumpSensor_t sensor)
{
    return RobotPlugPlay_IsBumpADCEnabled((uint8_t) sensor) ?
            AD_ReadADPin(BumpADPin(sensor)) : 0u;
}

uint16_t RobotSensors_ReadShooterMotorADC(void)
{
#if ROBOT_PLUGPLAY_USE_SHOOTER_ADC
    return AD_ReadADPin(SHOOTER_MOTOR_ADC_PIN);
#else
    return 0u;
#endif
}

uint8_t RobotSensors_IsTapeOn(TapeSensor_t sensor)
{
    if (RobotPlugPlay_IsTapeADCEnabled((uint8_t) sensor) == FALSE) {
        return FALSE;
    }
    return IsThresholdActive(RobotSensors_ReadTapeADC(sensor),
            TAPE_BLACK_ADC_THRESHOLD, TAPE_BLACK_IS_HIGH);
}

uint8_t RobotSensors_IsSolenoidOn(SolenoidSensor_t sensor)
{
    if (RobotPlugPlay_IsSolenoidADCEnabled((uint8_t) sensor) == FALSE) {
        return FALSE;
    }
    return IsThresholdActive(RobotSensors_ReadSolenoidADC(sensor),
            SOLENOID_ON_ADC_THRESHOLD, SOLENOID_ON_IS_HIGH);
}

uint8_t RobotSensors_IsBumpOn(BumpSensor_t sensor)
{
    if (RobotPlugPlay_IsBumpADCEnabled((uint8_t) sensor) == FALSE) {
        return FALSE;
    }
    return IsThresholdActive(RobotSensors_ReadBumpADC(sensor),
            BUMP_ON_ADC_THRESHOLD, BUMP_ON_IS_HIGH);
}

static unsigned int TapeADPin(TapeSensor_t sensor)
{
    switch (sensor) {
    case TAPE_SENSOR_1:
        return TAPE_SENSOR_1_ADC_PIN;
    case TAPE_SENSOR_2:
        return TAPE_SENSOR_2_ADC_PIN;
    case TAPE_SENSOR_3:
        return TAPE_SENSOR_3_ADC_PIN;
    case TAPE_SENSOR_4:
        return TAPE_SENSOR_4_ADC_PIN;
    case TAPE_SENSOR_5:
        return TAPE_SENSOR_5_ADC_PIN;
    case TAPE_SENSOR_6:
        return TAPE_SENSOR_6_ADC_PIN;
    case TAPE_SENSOR_7:
        return TAPE_SENSOR_7_ADC_PIN;
    case TAPE_SENSOR_8:
        return TAPE_SENSOR_8_ADC_PIN;
    case TAPE_SENSOR_9:
        return TAPE_SENSOR_9_ADC_PIN;
    default:
        return TAPE_SENSOR_1_ADC_PIN;
    }
}

static unsigned int SolenoidADPin(SolenoidSensor_t sensor)
{
    switch (sensor) {
    case SOLENOID_SENSOR_1:
        return SOLENOID_SENSOR_1_ADC_PIN;
    case SOLENOID_SENSOR_2:
        return SOLENOID_SENSOR_2_ADC_PIN;
    case SOLENOID_SENSOR_3:
        return SOLENOID_SENSOR_3_ADC_PIN;
    case SOLENOID_SENSOR_4:
        return SOLENOID_SENSOR_4_ADC_PIN;
    case SOLENOID_SENSOR_5:
        return SOLENOID_SENSOR_5_ADC_PIN;
    case SOLENOID_SENSOR_6:
        return SOLENOID_SENSOR_6_ADC_PIN;
    default:
        return SOLENOID_SENSOR_1_ADC_PIN;
    }
}

static unsigned int BumpADPin(BumpSensor_t sensor)
{
    switch (sensor) {
    case BUMP_SENSOR_1:
        return BUMP_SENSOR_1_ADC_PIN;
    case BUMP_SENSOR_2:
        return BUMP_SENSOR_2_ADC_PIN;
    case BUMP_SENSOR_3:
        return BUMP_SENSOR_3_ADC_PIN;
    case BUMP_SENSOR_4:
        return BUMP_SENSOR_4_ADC_PIN;
    default:
        return BUMP_SENSOR_1_ADC_PIN;
    }
}

static uint8_t IsThresholdActive(uint16_t reading, uint16_t threshold, uint8_t activeHigh)
{
    if (activeHigh) {
        return (reading >= threshold) ? TRUE : FALSE;
    }
    return (reading <= threshold) ? TRUE : FALSE;
}
