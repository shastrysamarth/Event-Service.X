#include "RobotEventCheckers.h"

#include "AlignSubHSM.h"
#include "ES_Configure.h"
#include "ES_Events.h"
#include "FindFrontTapeSubHSM.h"
#include "NavigateToISZSubHSM.h"
#include "RobotHSM.h"
#include "RobotIMU.h"
#include "RobotMotion.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"
#include "RobotSensors.h"
#include "RobotTestHarness.h"
#include "ShootingSubHSM.h"

typedef struct {
    uint8_t tape[5];
    uint8_t solenoid[6];
    uint8_t bump[4];
    uint8_t tape2On3Off;
    uint8_t tape2On4Off;
    uint8_t tape2Off3On;
    uint8_t tape2Off4On;
    uint8_t misaligned;
} SensorSnapshot_t;

static SensorSnapshot_t last;
static uint8_t initialized = FALSE;
static uint16_t lastBeaconADC = 0u;
static uint16_t peakBeaconADC = 0u;
static uint8_t beaconHadIncrease = FALSE;

static uint8_t PostEvent(ES_EventTyp_t eventType, uint16_t eventParam);
static uint8_t PostOnRising(uint8_t current, uint8_t *previous,
        ES_EventTyp_t eventType, uint16_t eventParam);
static uint8_t PostOnChange(uint8_t current, uint8_t *previous,
        ES_EventTyp_t onEvent, ES_EventTyp_t offEvent, uint16_t eventParam);
static uint8_t IsBeaconSearchActive(void);
static uint8_t IsRobotMisaligned(void);
static float AbsFloat(float value);

void InitRobotEventCheckers(void)
{
    uint8_t i;

    for (i = 0; i < 5u; i++) {
        last.tape[i] = RobotSensors_IsTapeOn((TapeSensor_t) (i + 1u));
    }
    for (i = 0; i < 6u; i++) {
        last.solenoid[i] = RobotSensors_IsSolenoidOn((SolenoidSensor_t) (i + 1u));
    }
    for (i = 0; i < 4u; i++) {
        last.bump[i] = RobotSensors_IsBumpOn((BumpSensor_t) (i + 1u));
    }

    last.tape2On3Off = FALSE;
    last.tape2On4Off = FALSE;
    last.tape2Off3On = FALSE;
    last.tape2Off4On = FALSE;
    last.misaligned = FALSE;

    lastBeaconADC = RobotSensors_ReadBeaconADC();
    peakBeaconADC = lastBeaconADC;
    beaconHadIncrease = FALSE;
    initialized = TRUE;
}

uint8_t CheckRobotPeriodic(void)
{
    if (initialized == FALSE) {
        InitRobotEventCheckers();
    }

#ifdef ROBOT_KEYBOARD_TEST
    if (RobotTestHarness_CheckKeyboard() == TRUE) {
        return TRUE;
    }
#endif

    RobotIMU_Update();
    RobotIMU_DebugStreamTick();
    return FALSE;
}

uint8_t CheckBeaconEvents(void)
{
    uint16_t current;

#if !ROBOT_PLUGPLAY_USE_BEACON_ADC
    return FALSE;
#endif

    if (initialized == FALSE) {
        InitRobotEventCheckers();
    }

    current = RobotSensors_ReadBeaconADC();

    if (IsBeaconSearchActive() == FALSE) {
        lastBeaconADC = current;
        peakBeaconADC = current;
        beaconHadIncrease = FALSE;
        return FALSE;
    }

    if (current > peakBeaconADC) {
        peakBeaconADC = current;
    }

    if (current >= (lastBeaconADC + BEACON_ADC_DELTA)) {
        lastBeaconADC = current;
        beaconHadIncrease = TRUE;
        return PostEvent(BeaconADCIncreaseEvent, current);
    }

    if ((beaconHadIncrease == TRUE) &&
            ((current + BEACON_ADC_DELTA) <= peakBeaconADC)) {
        uint16_t peak = peakBeaconADC;

        lastBeaconADC = current;
        peakBeaconADC = current;
        beaconHadIncrease = FALSE;
        return PostEvent(MaxSignalFoundEvent, peak);
    }

    lastBeaconADC = current;
    return FALSE;
}

uint8_t CheckCompoundNavigationEvents(void)
{
    uint8_t tape2 = RobotSensors_IsTapeOn(TAPE_SENSOR_2);
    uint8_t tape3 = RobotSensors_IsTapeOn(TAPE_SENSOR_3);
    uint8_t tape4 = RobotSensors_IsTapeOn(TAPE_SENSOR_4);
    uint8_t current;

#if !ROBOT_PLUGPLAY_USE_ANY_TAPE
    return FALSE;
#endif

    if (initialized == FALSE) {
        InitRobotEventCheckers();
    }

    current = (tape2 && !tape3) ? TRUE : FALSE;
    if (PostOnRising(current, &last.tape2On3Off, TapeSensor2On3OffEvent, 0u)) {
        return TRUE;
    }

    current = (tape2 && !tape4) ? TRUE : FALSE;
    if (PostOnRising(current, &last.tape2On4Off, TapeSensor2On4OffEvent, 0u)) {
        return TRUE;
    }

    current = (!tape2 && tape3) ? TRUE : FALSE;
    if (PostOnRising(current, &last.tape2Off3On, TapeSensor2Off3OnEvent, 0u)) {
        return TRUE;
    }

    current = (!tape2 && tape4) ? TRUE : FALSE;
    if (PostOnRising(current, &last.tape2Off4On, TapeSensor2Off4OnEvent, 0u)) {
        return TRUE;
    }

    return FALSE;
}

uint8_t CheckTapeEvents(void)
{
    ES_EventTyp_t onEvents[5] = {
        TapeSensor1OnEvent,
        TapeSensor2OnEvent,
        TapeSensor3OnEvent,
        TapeSensor4OnEvent,
        TapeSensor5OnEvent,
    };
    ES_EventTyp_t offEvents[5] = {
        TapeSensor1OffEvent,
        TapeSensor2OffEvent,
        TapeSensor3OffEvent,
        TapeSensor4OffEvent,
        TapeSensor5OffEvent,
    };
    uint8_t i;
    uint8_t current;

#if !ROBOT_PLUGPLAY_USE_ANY_TAPE
    return FALSE;
#endif

    if (initialized == FALSE) {
        InitRobotEventCheckers();
    }

    for (i = 0; i < 5u; i++) {
        current = RobotSensors_IsTapeOn((TapeSensor_t) (i + 1u));
        if ((i == 4u) && (current == TRUE) && (last.tape[i] == FALSE) &&
                (NavigateToISZ_IsCountingTape5() == TRUE)) {
            last.tape[i] = current;
            return PostEvent(TapeSensor5LowToHighEvent, 5u);
        }
        if (PostOnChange(current, &last.tape[i], onEvents[i], offEvents[i], (uint16_t) (i + 1u))) {
            return TRUE;
        }
    }

    return FALSE;
}

uint8_t CheckSolenoidEvents(void)
{
    ES_EventTyp_t onEvents[6] = {
        Solenoid1OnEvent,
        Solenoid2OnEvent,
        Solenoid3OnEvent,
        Solenoid4OnEvent,
        Solenoid5OnEvent,
        Solenoid6OnEvent,
    };
    uint8_t i;
    uint8_t current;

#if !ROBOT_PLUGPLAY_USE_ANY_SOLENOID_ADC
    return FALSE;
#endif

    if (initialized == FALSE) {
        InitRobotEventCheckers();
    }

    for (i = 0; i < 6u; i++) {
        current = RobotSensors_IsSolenoidOn((SolenoidSensor_t) (i + 1u));
        if (PostOnRising(current, &last.solenoid[i], onEvents[i], (uint16_t) (i + 1u))) {
            return TRUE;
        }
    }

    return FALSE;
}

uint8_t CheckBumpEvents(void)
{
    ES_EventTyp_t onEvents[4] = {
        BumpSensor1OnEvent,
        BumpSensor2OnEvent,
        BumpSensor3OnEvent,
        BumpSensor4OnEvent,
    };
    ES_EventTyp_t offEvents[4] = {
        BumpSensor1OffEvent,
        BumpSensor2OffEvent,
        BumpSensor3OffEvent,
        BumpSensor4OffEvent,
    };
    uint8_t i;
    uint8_t current;

#if !ROBOT_PLUGPLAY_USE_ANY_BUMP_ADC
    return FALSE;
#endif

    if (initialized == FALSE) {
        InitRobotEventCheckers();
    }

    for (i = 0; i < 4u; i++) {
        current = RobotSensors_IsBumpOn((BumpSensor_t) (i + 1u));
        if (PostOnChange(current, &last.bump[i], onEvents[i], offEvents[i], (uint16_t) (i + 1u))) {
            return TRUE;
        }
    }

    return FALSE;
}

uint8_t CheckDistanceMove(void)
{
    if (RobotMotion_IsDistanceMoveActive() &&
            RobotMotion_IsDistanceMoveComplete()) {
        return PostEvent(DistanceMoveCompleteEvent, 0u);
    }

    return FALSE;
}

uint8_t CheckAlignEvents(void)
{
#if !ROBOT_PLUGPLAY_USE_BNO055
    return FALSE;
#endif

    if (NavigateToISZ_IsAligning() == FALSE) {
        return FALSE;
    }

    AlignSubHSM_UpdateControl();

    /* DEPRECATED: position-align completion (PositionRealignedEvent) no longer posted here. */
#if 0
    if (AlignSubHSM_IsPositionStage() && AlignSubHSM_IsPositionAligned()) {
        return PostEvent(PositionRealignedEvent, 0u);
    }
#endif

    if (AlignSubHSM_IsHeadingStage() && AlignSubHSM_IsHeadingAligned()) {
        return PostEvent(RealignedEvent, ALIGN_REALIGNED_SOURCE_SENSOR);
    }

    return FALSE;
}

uint8_t CheckMisalignment(void)
{
    uint8_t current;

#if !ROBOT_PLUGPLAY_USE_BNO055
    return FALSE;
#endif

    if (NavigateToISZ_AllowsAlign() == FALSE) {
        last.misaligned = FALSE;
        return FALSE;
    }

    current = IsRobotMisaligned();
    return PostOnRising(current, &last.misaligned, MisalignedEvent, 0u);
}

static uint8_t PostOnRising(uint8_t current, uint8_t *previous,
        ES_EventTyp_t eventType, uint16_t eventParam)
{
    if ((current == TRUE) && (*previous == FALSE)) {
        *previous = current;
        return PostEvent(eventType, eventParam);
    }

    *previous = current;
    return FALSE;
}

static uint8_t PostOnChange(uint8_t current, uint8_t *previous,
        ES_EventTyp_t onEvent, ES_EventTyp_t offEvent, uint16_t eventParam)
{
    if (current == *previous) {
        return FALSE;
    }

    *previous = current;
    return PostEvent(current ? onEvent : offEvent, eventParam);
}

static uint8_t PostEvent(ES_EventTyp_t eventType, uint16_t eventParam)
{
    ES_Event event;

    event.EventType = eventType;
    event.EventParam = eventParam;
    return PostRobotHSM(event);
}

static uint8_t IsBeaconSearchActive(void)
{
    return (FindFrontTape_IsBeaconSearchActive() ||
            ShootingSubHSM_IsBeaconSearchActive()) ? TRUE : FALSE;
}

static uint8_t IsRobotMisaligned(void)
{
    float headingError = RobotIMU_GetHeadingErrorToZeroDeg();

    /* Heading-only misalignment (DEPRECATED: position vs ref was IMU-based). */
    return (AbsFloat(headingError) > HEADING_THRESHOLD_DEG) ? TRUE : FALSE;
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}
