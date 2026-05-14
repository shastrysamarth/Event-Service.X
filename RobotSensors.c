#include "RobotSensors.h"

#include "AD.h"
#include "IO_Ports.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"

static uint16_t TapeDigitalPin(TapeSensor_t sensor);
static unsigned int SolenoidADPin(SolenoidSensor_t sensor);
static uint8_t BumpDigitalPort(BumpSensor_t sensor);
static uint16_t BumpDigitalPin(BumpSensor_t sensor);
static uint8_t IsThresholdActive(uint16_t reading, uint16_t threshold, uint8_t activeHigh);

uint8_t RobotSensors_Init(void)
{
    unsigned int pins = 0u;

#if ROBOT_PLUGPLAY_USE_ANY_TAPE
    IO_PortsSetPortInputs(PORTV, TAPE_SENSOR_PORTV_PINS);
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
#if ROBOT_PLUGPLAY_USE_BUMP1 || ROBOT_PLUGPLAY_USE_BUMP2 || ROBOT_PLUGPLAY_USE_BUMP3
    IO_PortsSetPortInputs(PORTW, BUMP_SENSOR_PORTW_PINS);
#endif
#if ROBOT_PLUGPLAY_USE_BUMP4
    IO_PortsSetPortInputs(PORTV, BUMP_SENSOR_4_PIN);
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

uint8_t RobotSensors_ReadTapeDigital(TapeSensor_t sensor)
{
    uint16_t pin;

    if (RobotPlugPlay_IsTapeEnabled((uint8_t) sensor) == FALSE) {
        return 0u;
    }
    pin = TapeDigitalPin(sensor);
    return (IO_PortsReadPort(PORTV) & pin) ? 1u : 0u;
}

uint16_t RobotSensors_ReadSolenoidADC(SolenoidSensor_t sensor)
{
    return RobotPlugPlay_IsSolenoidADCEnabled((uint8_t) sensor) ?
            AD_ReadADPin(SolenoidADPin(sensor)) : 0u;
}

uint8_t RobotSensors_ReadBumpDigital(BumpSensor_t sensor)
{
    uint8_t port;
    uint16_t pin;

    if (RobotPlugPlay_IsBumpEnabled((uint8_t) sensor) == FALSE) {
        return 0u;
    }
    port = BumpDigitalPort(sensor);
    pin = BumpDigitalPin(sensor);
    return (IO_PortsReadPort(port) & pin) ? 1u : 0u;
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
    uint8_t reading;

    if (RobotPlugPlay_IsTapeEnabled((uint8_t) sensor) == FALSE) {
        return FALSE;
    }
    reading = RobotSensors_ReadTapeDigital(sensor);
    return TAPE_BLACK_IS_HIGH ? (reading != 0u) : (reading == 0u);
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
    uint8_t reading;

    if (RobotPlugPlay_IsBumpEnabled((uint8_t) sensor) == FALSE) {
        return FALSE;
    }
    reading = RobotSensors_ReadBumpDigital(sensor);
    return BUMP_ON_IS_HIGH ? (reading != 0u) : (reading == 0u);
}

static uint16_t TapeDigitalPin(TapeSensor_t sensor)
{
    switch (sensor) {
    case TAPE_SENSOR_1:
        return TAPE_SENSOR_1_PIN;
    case TAPE_SENSOR_2:
        return TAPE_SENSOR_2_PIN;
    case TAPE_SENSOR_3:
        return TAPE_SENSOR_3_PIN;
    case TAPE_SENSOR_4:
        return TAPE_SENSOR_4_PIN;
    case TAPE_SENSOR_5:
        return TAPE_SENSOR_5_PIN;
    default:
        return 0u;
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

static uint8_t BumpDigitalPort(BumpSensor_t sensor)
{
    switch (sensor) {
    case BUMP_SENSOR_1:
        return BUMP_SENSOR_1_PORT;
    case BUMP_SENSOR_2:
        return BUMP_SENSOR_2_PORT;
    case BUMP_SENSOR_3:
        return BUMP_SENSOR_3_PORT;
    case BUMP_SENSOR_4:
        return BUMP_SENSOR_4_PORT;
    default:
        return BUMP_SENSOR_1_PORT;
    }
}

static uint16_t BumpDigitalPin(BumpSensor_t sensor)
{
    switch (sensor) {
    case BUMP_SENSOR_1:
        return BUMP_SENSOR_1_PIN;
    case BUMP_SENSOR_2:
        return BUMP_SENSOR_2_PIN;
    case BUMP_SENSOR_3:
        return BUMP_SENSOR_3_PIN;
    case BUMP_SENSOR_4:
        return BUMP_SENSOR_4_PIN;
    default:
        return 0u;
    }
}

static uint8_t IsThresholdActive(uint16_t reading, uint16_t threshold, uint8_t activeHigh)
{
    if (activeHigh) {
        return (reading >= threshold) ? TRUE : FALSE;
    }
    return (reading <= threshold) ? TRUE : FALSE;
}
