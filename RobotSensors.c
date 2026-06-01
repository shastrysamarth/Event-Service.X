#include "RobotSensors.h"

#include <xc.h>
#include <sys/attribs.h>

#include "AD.h"
#include "BOARD.h"
#include "IO_Ports.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"

#define SENSOR_SAMPLER_HZ 1000u

static uint8_t TapeDigitalPort(TapeSensor_t sensor);
static uint16_t TapeDigitalPin(TapeSensor_t sensor);
static uint8_t TapeOnIsHigh(TapeSensor_t sensor);
static uint8_t BumpDigitalPort(BumpSensor_t sensor);
static uint16_t BumpDigitalPin(BumpSensor_t sensor);
static uint16_t BeaconAveragePush(uint16_t reading);
static void RobotSensors_SampleNow(void);
static void RobotSensors_InitSampler(void);

static volatile uint8_t sampledTapeRaw[5];
static volatile uint8_t sampledTapeOn[5];
static volatile uint8_t sampledBumpRaw[4];
static volatile uint8_t sampledBumpOn[4];
static volatile uint16_t beaconSamples[BEACON_AVERAGE_SAMPLE_COUNT];
static volatile uint32_t beaconSampleSum = 0u;
static volatile uint8_t beaconSampleIndex = 0u;
static volatile uint8_t beaconSampleCount = 0u;
static volatile uint16_t sampledBeaconRawADC = 0u;
static volatile uint16_t sampledBeaconSmoothADC = 0u;
static volatile uint32_t sensorSampleTicks = 0u;

uint8_t RobotSensors_Init(void)
{
    unsigned int pins = 0u;

#if ROBOT_PLUGPLAY_USE_ANY_TAPE
    IO_PortsSetPortInputs(PORTZ, TAPE_SENSOR_PORTZ_PINS);
    IO_PortsSetPortInputs(PORTW, TAPE_SENSOR_PORTW_PINS);
#endif
#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    pins |= BEACON_ADC_PIN;
#endif
    /* Solenoid ADC placements intentionally disabled for now. */
#if ROBOT_PLUGPLAY_USE_BUMP1 || ROBOT_PLUGPLAY_USE_BUMP3 || ROBOT_PLUGPLAY_USE_BUMP4
    IO_PortsSetPortInputs(PORTV, BUMP_SENSOR_PORTV_PINS);
#endif
#if ROBOT_PLUGPLAY_USE_BUMP2
    IO_PortsSetPortInputs(PORTZ, BUMP_SENSOR_PORTZ_PINS);
#endif
#if ROBOT_PLUGPLAY_USE_SHOOTER_ADC
    pins |= SHOOTER_MOTOR_ADC_PIN;
#endif

    if (pins == 0u) {
        RobotSensors_InitSampler();
        return TRUE;
    }
    if (AD_AddPins(pins) != SUCCESS) {
        return FALSE;
    }
    RobotSensors_InitSampler();
    return TRUE;
}

uint16_t RobotSensors_ReadBeaconRawADC(void)
{
#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    return AD_ReadADPin(BEACON_ADC_PIN);
#else
    return 0u;
#endif
}

uint16_t RobotSensors_GetBeaconRawADC(void)
{
#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    return sampledBeaconRawADC;
#else
    return 0u;
#endif
}

uint16_t RobotSensors_ReadBeaconADC(void)
{
#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    return RobotSensors_PushBeaconADC(RobotSensors_ReadBeaconRawADC());
#else
    return 0u;
#endif
}

uint16_t RobotSensors_PushBeaconADC(uint16_t reading)
{
#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    sampledBeaconRawADC = reading;
    sampledBeaconSmoothADC = BeaconAveragePush(reading);
    return sampledBeaconSmoothADC;
#else
    (void)reading;
    return 0u;
#endif
}

uint16_t RobotSensors_GetBeaconADC(void)
{
#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    return sampledBeaconSmoothADC;
#else
    return 0u;
#endif
}

uint8_t RobotSensors_ReadTapeDigital(TapeSensor_t sensor)
{
    uint8_t port;
    uint16_t pin;

    if (RobotPlugPlay_IsTapeEnabled((uint8_t) sensor) == FALSE) {
        return 0u;
    }
    port = TapeDigitalPort(sensor);
    pin = TapeDigitalPin(sensor);
    return (IO_PortsReadPort(port) & pin) ? 1u : 0u;
}

uint8_t RobotSensors_GetTapeDigital(TapeSensor_t sensor)
{
#if ROBOT_PLUGPLAY_USE_ANY_TAPE
    if (RobotPlugPlay_IsTapeEnabled((uint8_t) sensor) == FALSE) {
        return 0u;
    }
    if ((sensor < TAPE_SENSOR_1) || (sensor > TAPE_SENSOR_5)) {
        return 0u;
    }
    return sampledTapeRaw[(uint8_t) sensor - 1u];
#else
    (void)sensor;
    return 0u;
#endif
}

uint16_t RobotSensors_ReadSolenoidADC(SolenoidSensor_t sensor)
{
    (void) sensor;
    return 0u;
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

uint8_t RobotSensors_GetBumpDigital(BumpSensor_t sensor)
{
#if ROBOT_PLUGPLAY_USE_ANY_BUMP
    if (RobotPlugPlay_IsBumpEnabled((uint8_t) sensor) == FALSE) {
        return 0u;
    }
    if ((sensor < BUMP_SENSOR_1) || (sensor > BUMP_SENSOR_4)) {
        return 0u;
    }
    return sampledBumpRaw[(uint8_t) sensor - 1u];
#else
    (void)sensor;
    return 0u;
#endif
}

uint16_t RobotSensors_ReadShooterMotorADC(void)
{
#if ROBOT_PLUGPLAY_USE_SHOOTER_ADC
    return AD_ReadADPin(SHOOTER_MOTOR_ADC_PIN);
#else
    return 0u;
#endif
}

uint8_t RobotSensors_ReadBeaconDistanceFeet(void)
{
#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    return RobotSensors_BeaconDistanceFeetFromADC(RobotSensors_GetBeaconADC());
#else
    return BEACON_DISTANCE_UNKNOWN_FT;
#endif
}

uint8_t RobotSensors_BeaconDistanceFeetFromADC(uint16_t reading)
{
    uint32_t numerator;
    uint32_t denominator;

    if (reading >= BEACON_ADC_AT_6FT) {
        return BEACON_DISTANCE_MIN_FT;
    }
    if (reading <= BEACON_ADC_AT_16FT) {
        return BEACON_DISTANCE_MAX_FT;
    }

    numerator = ((uint32_t) (BEACON_ADC_AT_6FT - reading)) *
            (BEACON_DISTANCE_MAX_FT - BEACON_DISTANCE_MIN_FT);
    denominator = (uint32_t) (BEACON_ADC_AT_6FT - BEACON_ADC_AT_16FT);
    return (uint8_t) (BEACON_DISTANCE_MIN_FT +
            ((numerator + (denominator / 2u)) / denominator));
}

uint8_t RobotSensors_IsTapeOn(TapeSensor_t sensor)
{
#if ROBOT_PLUGPLAY_USE_ANY_TAPE
    if (RobotPlugPlay_IsTapeEnabled((uint8_t) sensor) == FALSE) {
        return FALSE;
    }
    if ((sensor < TAPE_SENSOR_1) || (sensor > TAPE_SENSOR_5)) {
        return FALSE;
    }
    return sampledTapeOn[(uint8_t) sensor - 1u];
#else
    (void)sensor;
    return FALSE;
#endif
}

uint8_t RobotSensors_TapeRawIsOn(TapeSensor_t sensor, uint8_t rawReading)
{
    return TapeOnIsHigh(sensor) ? (rawReading != 0u) : (rawReading == 0u);
}

uint8_t RobotSensors_IsSolenoidOn(SolenoidSensor_t sensor)
{
    (void) sensor;
    return FALSE;
}

uint8_t RobotSensors_IsBumpOn(BumpSensor_t sensor)
{
#if ROBOT_PLUGPLAY_USE_ANY_BUMP
    if (RobotPlugPlay_IsBumpEnabled((uint8_t) sensor) == FALSE) {
        return FALSE;
    }
    if ((sensor < BUMP_SENSOR_1) || (sensor > BUMP_SENSOR_4)) {
        return FALSE;
    }
    return sampledBumpOn[(uint8_t) sensor - 1u];
#else
    (void)sensor;
    return FALSE;
#endif
}

uint8_t RobotSensors_BumpRawIsOn(uint8_t rawReading)
{
    return BUMP_ON_IS_HIGH ? (rawReading != 0u) : (rawReading == 0u);
}

static uint8_t TapeDigitalPort(TapeSensor_t sensor)
{
    switch (sensor) {
    case TAPE_SENSOR_1:
        return TAPE_SENSOR_1_PORT;
    case TAPE_SENSOR_2:
        return TAPE_SENSOR_2_PORT;
    case TAPE_SENSOR_3:
        return TAPE_SENSOR_3_PORT;
    case TAPE_SENSOR_4:
        return TAPE_SENSOR_4_PORT;
    case TAPE_SENSOR_5:
        return TAPE_SENSOR_5_PORT;
    default:
        return TAPE_SENSOR_1_PORT;
    }
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

static uint8_t TapeOnIsHigh(TapeSensor_t sensor)
{
    switch (sensor) {
    case TAPE_SENSOR_1:
        return TAPE_SENSOR_1_ON_IS_HIGH;
    case TAPE_SENSOR_2:
        return TAPE_SENSOR_2_ON_IS_HIGH;
    case TAPE_SENSOR_3:
        return TAPE_SENSOR_3_ON_IS_HIGH;
    case TAPE_SENSOR_4:
        return TAPE_SENSOR_4_ON_IS_HIGH;
    case TAPE_SENSOR_5:
        return TAPE_SENSOR_5_ON_IS_HIGH;
    default:
        return TAPE_BLACK_IS_HIGH;
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

static uint16_t BeaconAveragePush(uint16_t reading)
{
    if (beaconSampleCount < BEACON_AVERAGE_SAMPLE_COUNT) {
        beaconSamples[beaconSampleIndex] = reading;
        beaconSampleSum += reading;
        beaconSampleCount++;
    } else {
        beaconSampleSum -= beaconSamples[beaconSampleIndex];
        beaconSamples[beaconSampleIndex] = reading;
        beaconSampleSum += reading;
    }

    beaconSampleIndex++;
    if (beaconSampleIndex >= BEACON_AVERAGE_SAMPLE_COUNT) {
        beaconSampleIndex = 0u;
    }

    return (uint16_t) ((beaconSampleSum + (beaconSampleCount / 2u)) /
            beaconSampleCount);
}

static void RobotSensors_InitSampler(void)
{
    RobotSensors_SampleNow();

    T3CON = 0;
    TMR3 = 0;
    PR3 = BOARD_GetPBClock() / SENSOR_SAMPLER_HZ;
    IFS0bits.T3IF = 0;
    IPC3bits.T3IP = 5;
    IPC3bits.T3IS = 0;
    IEC0bits.T3IE = 1;
    T3CONbits.ON = 1;
}

static void RobotSensors_SampleNow(void)
{
#if ROBOT_PLUGPLAY_USE_ANY_TAPE || ROBOT_PLUGPLAY_USE_ANY_BUMP
    uint8_t i;
    uint8_t raw;
#endif

#if ROBOT_PLUGPLAY_USE_ANY_TAPE
    for (i = 0u; i < 5u; i++) {
        if (RobotPlugPlay_IsTapeEnabled(i + 1u) == FALSE) {
            sampledTapeRaw[i] = 0u;
            sampledTapeOn[i] = FALSE;
            continue;
        }
        raw = RobotSensors_ReadTapeDigital((TapeSensor_t)(i + 1u));
        sampledTapeRaw[i] = raw;
        sampledTapeOn[i] = RobotSensors_TapeRawIsOn((TapeSensor_t)(i + 1u), raw);
    }
#endif

#if ROBOT_PLUGPLAY_USE_ANY_BUMP
    for (i = 0u; i < 4u; i++) {
        if (RobotPlugPlay_IsBumpEnabled(i + 1u) == FALSE) {
            sampledBumpRaw[i] = 0u;
            sampledBumpOn[i] = FALSE;
            continue;
        }
        raw = RobotSensors_ReadBumpDigital((BumpSensor_t)(i + 1u));
        sampledBumpRaw[i] = raw;
        sampledBumpOn[i] = RobotSensors_BumpRawIsOn(raw);
    }
#endif

#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    (void)RobotSensors_PushBeaconADC(RobotSensors_ReadBeaconRawADC());
#endif

    sensorSampleTicks++;
}

void __ISR(_TIMER_3_VECTOR, IPL5SOFT) RobotSensorsTimer3IntHandler(void)
{
    IFS0bits.T3IF = 0;
    RobotSensors_SampleNow();
}
