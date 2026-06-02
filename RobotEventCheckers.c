#include "RobotEventCheckers.h"

#include "AlignSubHSM.h"
#include "ES_Configure.h"
#include "ES_Events.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
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

#include <stdint.h>
#include <stdio.h>
#define max(a,b) (((a) > (b)) ? (a) : (b))

#define TAPE_DEBUG_SAMPLE_PERIOD_MS 0u
#define BEACON_STREAM_PRINT_PERIOD_MS 250u

typedef struct
{
    uint8_t tape[5];
    uint8_t solenoid[6];
    uint8_t bump[4];
    uint8_t misaligned;
} SensorSnapshot_t;

static SensorSnapshot_t last;
static uint8_t initialized = FALSE;
static uint16_t lastBeaconADC = 0u;
static uint16_t peakBeaconADC = 0u;
static uint16_t minBeaconADC = 1024u;
static uint8_t beaconHadIncrease = FALSE;
static uint32_t lastTapeSampleLogMs = 0u;
static uint8_t beaconStreamActive = FALSE;
static uint32_t lastBeaconStreamPrintMs = 0u;
static uint32_t lastImuUpdateMs = 0u;

static void BeaconStreamTick(void);
static uint8_t PostEvent(ES_EventTyp_t eventType, uint16_t eventParam);
static uint8_t PostOnRising(uint8_t current, uint8_t *previous,
                            ES_EventTyp_t eventType, uint16_t eventParam);
static uint8_t PostOnChange(uint8_t current, uint8_t *previous,
                            ES_EventTyp_t onEvent, ES_EventTyp_t offEvent, uint16_t eventParam);
static uint8_t IsBeaconSearchActive(void);
static uint8_t IsRobotMisaligned(void);
static float AbsFloat(float value);
static void LogIMUInitial(void);
static void LogIMUEvent(ES_EventTyp_t eventType);
static void LogBeaconInitial(uint16_t raw, uint16_t smoothed);
static void LogBeaconEvent(ES_EventTyp_t eventType, uint16_t current,
                           uint16_t peak);
static void LogShooterADCInitial(uint16_t raw);
static void LogTapeInitial(uint8_t sensorNumber, uint8_t raw, uint8_t onTape);
static void LogTapeChange(uint8_t sensorNumber, uint8_t raw, uint8_t onTape);
static void LogTapeMask(uint8_t tapeMask, uint8_t changedMask);
static void LogTapeSample(const uint8_t raw[5], const uint8_t onTape[5]);
static uint8_t TapeMaskFromReadings(const uint8_t onTape[5]);
static uint8_t BumpMaskFromReadings(const uint8_t bumped[4]);
static void LogBumpInitial(uint8_t sensorNumber, uint8_t raw, uint8_t bumped);
static void LogBumpChange(uint8_t sensorNumber, uint8_t raw, uint8_t bumped);
static void LogBumpMask(uint8_t bumpMask, uint8_t changedMask);
static void PrintFixedValue(const char *name, float value, const char *unit);

void InitRobotEventCheckers(void)
{
    uint8_t i;
    uint8_t raw;
    uint16_t beaconRaw;

#if ROBOT_PLUGPLAY_USE_BNO055
    LogIMUInitial();
#endif

    for (i = 0; i < 5u; i++)
    {
        if (RobotPlugPlay_IsTapeEnabled(i + 1u) == FALSE)
        {
            last.tape[i] = FALSE;
            continue;
        }
        raw = RobotSensors_ReadTapeDigital((TapeSensor_t)(i + 1u));
        last.tape[i] = RobotSensors_TapeRawIsOn((TapeSensor_t)(i + 1u), raw);
        LogTapeInitial(i + 1u, raw, last.tape[i]);
    }
    for (i = 0; i < 6u; i++)
    {
        last.solenoid[i] = RobotSensors_IsSolenoidOn((SolenoidSensor_t)(i + 1u));
    }
    for (i = 0; i < 4u; i++)
    {
        if (RobotPlugPlay_IsBumpEnabled(i + 1u) == FALSE)
        {
            last.bump[i] = FALSE;
            continue;
        }
        raw = RobotSensors_ReadBumpDigital((BumpSensor_t)(i + 1u));
        last.bump[i] = RobotSensors_BumpRawIsOn(raw);
        LogBumpInitial(i + 1u, raw, last.bump[i]);
    }

    last.misaligned = FALSE;

#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    beaconRaw = RobotSensors_GetBeaconRawADC();
    lastBeaconADC = RobotSensors_GetBeaconADC();
    LogBeaconInitial(beaconRaw, lastBeaconADC);
#else
    beaconRaw = 0u;
    lastBeaconADC = 0u;
#endif
    peakBeaconADC = lastBeaconADC;
    minBeaconADC = lastBeaconADC;
    beaconHadIncrease = FALSE;
    lastImuUpdateMs = ES_Timer_GetTime() - ROBOT_IMU_UPDATE_PERIOD_MS;
#if ROBOT_PLUGPLAY_USE_SHOOTER_ADC
    LogShooterADCInitial(RobotSensors_ReadShooterMotorADC());
#endif
    initialized = TRUE;
}

uint8_t RobotEventCheckers_GetRobotQueueMaxDepth(void)
{
    return ES_GetServiceMaxQueueDepth(0u);
}

uint16_t RobotEventCheckers_GetRobotQueuePostFailures(void)
{
    return ES_GetServicePostFailures(0u);
}

void RobotEventCheckers_ToggleBeaconStream(void)
{
    uint32_t nowMs;

#if !ROBOT_PLUGPLAY_USE_BEACON_ADC
    printf("[SENSOR] beacon stream unavailable; rebuild with HW_BEACON=1\r\n");
    return;
#endif

    nowMs = ES_Timer_GetTime();
    beaconStreamActive = !beaconStreamActive;
    if (beaconStreamActive)
    {
        printf("[SENSOR] beacon stream ON\r\n");
        lastBeaconStreamPrintMs = nowMs - BEACON_STREAM_PRINT_PERIOD_MS;
        BeaconStreamTick();
    }
    else
    {
        printf("[SENSOR] beacon stream OFF\r\n");
    }
}

static void BeaconStreamTick(void)
{
#if ROBOT_PLUGPLAY_USE_BEACON_ADC
    uint16_t smoothADC;
    uint16_t rawADC;
    uint32_t nowMs;

    if (beaconStreamActive == FALSE)
    {
        return;
    }
    nowMs = ES_Timer_GetTime();

    if ((uint32_t)(nowMs - lastBeaconStreamPrintMs) <
        BEACON_STREAM_PRINT_PERIOD_MS)
    {
        return;
    }
    lastBeaconStreamPrintMs = nowMs;

    rawADC = RobotSensors_GetBeaconRawADC();
    smoothADC = RobotSensors_GetBeaconADC();

    printf("[SENSOR] beacon rawADC=%u smoothADC=%u distance=%u ft pin=%s\r\n",
           (unsigned int)rawADC,
           (unsigned int)smoothADC,
           (unsigned int)RobotSensors_BeaconDistanceFeetFromADC(smoothADC),
           BEACON_ADC_PIN_LABEL);
#endif
}

uint8_t CheckRobotPeriodic(void)
{
    uint8_t keyboardHandled = FALSE;
    uint32_t nowMs;

    if (initialized == FALSE)
    {
        InitRobotEventCheckers();
    }

#ifdef ROBOT_KEYBOARD_TEST
    if (RobotTestHarness_CheckKeyboard() == TRUE)
    {
        keyboardHandled = TRUE;
    }
#endif

    nowMs = ES_Timer_GetTime();
    if ((uint32_t)(nowMs - lastImuUpdateMs) >= ROBOT_IMU_UPDATE_PERIOD_MS)
    {
        lastImuUpdateMs = nowMs;
        RobotIMU_Update();
    }
    RobotIMU_DebugStreamTick();
    BeaconStreamTick();
    return keyboardHandled;
}

uint8_t CheckBeaconEvents(void)
{
    uint16_t current;

#if !ROBOT_PLUGPLAY_USE_BEACON_ADC
    return FALSE;
#endif

    if (initialized == FALSE)
    {
        InitRobotEventCheckers();
    }

    current = RobotSensors_GetBeaconADC();

    if (IsBeaconSearchActive() == FALSE)
    {
        lastBeaconADC = current;
        peakBeaconADC = current;
        minBeaconADC = current;
        beaconHadIncrease = FALSE;
        return FALSE;
    }

    if (current > peakBeaconADC)
    {
        peakBeaconADC = current;
    }

    if (current < minBeaconADC)
    {
        minBeaconADC = current;
    }

    if ((beaconHadIncrease == FALSE) && 
        (current >= (minBeaconADC + BEACON_ADC_DELTA)))
    {
        lastBeaconADC = current;
        beaconHadIncrease = TRUE;
        LogBeaconEvent(BeaconADCIncreaseEvent, current, peakBeaconADC);
        return PostEvent(BeaconADCIncreaseEvent, current);
    }

    if ((beaconHadIncrease == TRUE) &&
        ((current + BEACON_ADC_DELTA) <= peakBeaconADC))
    {
        uint16_t peak = peakBeaconADC;

        lastBeaconADC = current;
        beaconHadIncrease = FALSE;
        LogBeaconEvent(MaxSignalFoundEvent, current, peak);
        return PostEvent(MaxSignalFoundEvent, peak);
    }

    lastBeaconADC = current;
    return FALSE;
}

uint8_t CheckTapeEvents(void)
{
    uint8_t i;
    uint8_t raw;
    uint8_t current;
    uint8_t rawReadings[5];
    uint8_t tapeReadings[5];
    uint8_t tapeMask;
    uint8_t previousMask;
    uint8_t changedMask;
    uint32_t now;
    uint8_t postedAny = FALSE;

#if !ROBOT_PLUGPLAY_USE_ANY_TAPE
    return FALSE;
#endif

    if (initialized == FALSE)
    {
        InitRobotEventCheckers();
    }

    for (i = 0; i < 5u; i++)
    {
        raw = RobotSensors_GetTapeDigital((TapeSensor_t)(i + 1u));
        current = RobotSensors_TapeRawIsOn((TapeSensor_t)(i + 1u), raw);
        rawReadings[i] = raw;
        tapeReadings[i] = current;
    }

    tapeMask = TapeMaskFromReadings(tapeReadings);
    previousMask = TapeMaskFromReadings(last.tape);
    changedMask = (uint8_t) (tapeMask ^ previousMask);

    if (changedMask != 0u)
    {
        for (i = 0; i < 5u; i++)
        {
            if ((changedMask & (uint8_t)(1u << i)) == 0u)
            {
                continue;
            }
            raw = rawReadings[i];
            current = tapeReadings[i];
            last.tape[i] = current;
            LogTapeChange(i + 1u, raw, current);
        }
        LogTapeMask(tapeMask, changedMask);
        postedAny = PostEvent(TapeChangedEvent,
                TAPE_EVENT_MAKE_PARAM(tapeMask, changedMask));
    }

    now = ES_Timer_GetTime();
    if ((TAPE_DEBUG_SAMPLE_PERIOD_MS > 0u) &&
        (postedAny == FALSE) &&
        ((uint32_t)(now - lastTapeSampleLogMs) >= TAPE_DEBUG_SAMPLE_PERIOD_MS))
    {
        lastTapeSampleLogMs = now;
        LogTapeSample(rawReadings, tapeReadings);
    }

    return postedAny;
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
    uint8_t raw;
    uint8_t current;

#if !ROBOT_PLUGPLAY_USE_ANY_SOLENOID_ADC
    return FALSE;
#endif

    if (initialized == FALSE)
    {
        InitRobotEventCheckers();
    }

    for (i = 0; i < 6u; i++)
    {
        current = RobotSensors_IsSolenoidOn((SolenoidSensor_t)(i + 1u));
        if (PostOnRising(current, &last.solenoid[i], onEvents[i], (uint16_t)(i + 1u)))
        {
            return TRUE;
        }
    }

    return FALSE;
}

uint8_t CheckBumpEvents(void)
{
    uint8_t i;
    uint8_t raw;
    uint8_t current;
    uint8_t rawReadings[4];
    uint8_t bumpReadings[4];
    uint8_t bumpMask;
    uint8_t previousMask;
    uint8_t changedMask;
    uint8_t postedAny = FALSE;

#if !ROBOT_PLUGPLAY_USE_ANY_BUMP
    return FALSE;
#endif

    if (initialized == FALSE)
    {
        InitRobotEventCheckers();
    }

    for (i = 0; i < 4u; i++)
    {
        raw = RobotSensors_GetBumpDigital((BumpSensor_t)(i + 1u));
        current = RobotSensors_BumpRawIsOn(raw);
        rawReadings[i] = raw;
        bumpReadings[i] = current;
    }

    bumpMask = BumpMaskFromReadings(bumpReadings);
    previousMask = BumpMaskFromReadings(last.bump);
    changedMask = (uint8_t) (bumpMask ^ previousMask);

    if (changedMask != 0u)
    {
        for (i = 0; i < 4u; i++)
        {
            if ((changedMask & (uint8_t)(1u << i)) == 0u)
            {
                continue;
            }
            raw = rawReadings[i];
            current = bumpReadings[i];
            last.bump[i] = current;
            LogBumpChange(i + 1u, raw, current);
        }
        LogBumpMask(bumpMask, changedMask);
        postedAny = PostEvent(BumpChangedEvent,
                BUMP_EVENT_MAKE_PARAM(bumpMask, changedMask));
    }

    return postedAny;
}

uint8_t CheckDistanceMove(void)
{
    if (RobotMotion_IsDistanceMoveActive() &&
        RobotMotion_IsDistanceMoveComplete())
    {
        return PostEvent(DistanceMoveCompleteEvent, 0u);
    }

    return FALSE;
}

uint8_t CheckAlignEvents(void)
{
#if !ROBOT_PLUGPLAY_USE_BNO055
    return FALSE;
#endif

    if ((NavigateToISZ_IsAligning() == FALSE) &&
        (ShootingSubHSM_IsAligning() == FALSE))
    {
        return FALSE;
    }

    AlignSubHSM_UpdateControl();

    /* DEPRECATED: position-align completion (PositionRealignedEvent) no longer posted here. */
#if 0
    if (AlignSubHSM_IsPositionStage() && AlignSubHSM_IsPositionAligned()) {
        return PostEvent(PositionRealignedEvent, 0u);
    }
#endif

    if (AlignSubHSM_IsHeadingStage() && AlignSubHSM_IsHeadingAligned())
    {
        LogIMUEvent(RealignedEvent);
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

    if ((NavigateToISZ_AllowsAlign() == FALSE) &&
        (ShootingSubHSM_AllowsAlign() == FALSE))
    {
        last.misaligned = FALSE;
        return FALSE;
    }

    current = IsRobotMisaligned();
    if ((current == TRUE) && (last.misaligned == FALSE))
    {
        LogIMUEvent(MisalignedEvent);
    }
    return PostOnRising(current, &last.misaligned, MisalignedEvent, 0u);
}

static uint8_t PostOnRising(uint8_t current, uint8_t *previous,
                            ES_EventTyp_t eventType, uint16_t eventParam)
{
    if ((current == TRUE) && (*previous == FALSE))
    {
        *previous = current;
        return PostEvent(eventType, eventParam);
    }

    *previous = current;
    return FALSE;
}

static uint8_t PostOnChange(uint8_t current, uint8_t *previous,
                            ES_EventTyp_t onEvent, ES_EventTyp_t offEvent, uint16_t eventParam)
{
    if (current == *previous)
    {
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
            ShootingSubHSM_IsBeaconSearchActive())
               ? TRUE
               : FALSE;
}

static uint8_t IsRobotMisaligned(void)
{
    float headingError = RobotIMU_GetHeadingErrorToRefDeg();

    /* Heading-only misalignment (DEPRECATED: position vs ref was IMU-based). */
    return (AbsFloat(headingError) > HEADING_THRESHOLD_DEG) ? TRUE : FALSE;
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void LogIMUInitial(void)
{
#if defined(DEBUG) || defined(ROBOT_DEBUG)
    printf("[IMU] init ready=%u stationary=%u ",
           (unsigned int)RobotIMU_IsReady(),
           (unsigned int)RobotIMU_IsStationary());
    PrintFixedValue("heading", RobotIMU_GetHeadingDeg(), "deg");
    printf(" ");
    PrintFixedValue("headingError", RobotIMU_GetHeadingErrorToZeroDeg(), "deg");
    printf("\r\n");
#endif
}

static void LogIMUEvent(ES_EventTyp_t eventType)
{
#if ROBOT_REALTIME_TRACE
    printf("[IMU] ");
    PrintFixedValue("headingError", RobotIMU_GetHeadingErrorToZeroDeg(), "deg");
    printf(" ");
    PrintFixedValue("zGyro", RobotIMU_GetZGyroDPS(), "dps");
    printf(" stationary=%u -> %s\r\n",
           (unsigned int)RobotIMU_IsStationary(),
           EventNames[eventType]);
#else
    (void)eventType;
#endif
}

static void LogBeaconInitial(uint16_t raw, uint16_t smoothed)
{
#if defined(DEBUG) || defined(ROBOT_DEBUG)
    printf("[BEACON] init raw=%u smoothed=%u distance=%u ft\r\n",
           (unsigned int)raw,
           (unsigned int)smoothed,
           (unsigned int)RobotSensors_BeaconDistanceFeetFromADC(smoothed));
#else
    (void)raw;
    (void)smoothed;
#endif
}

static void LogBeaconEvent(ES_EventTyp_t eventType, uint16_t current,
                           uint16_t peak)
{
#if ROBOT_REALTIME_TRACE
    printf("[BEACON] smoothed=%u peak=%u distance=%u ft -> %s\r\n",
           (unsigned int)current,
           (unsigned int)peak,
           (unsigned int)RobotSensors_BeaconDistanceFeetFromADC(current),
           EventNames[eventType]);
#else
    (void)eventType;
    (void)current;
    (void)peak;
#endif
}

static void LogShooterADCInitial(uint16_t raw)
{
#if defined(DEBUG) || defined(ROBOT_DEBUG)
    printf("[SHOOTER_ADC] init raw=%u\r\n", (unsigned int)raw);
#else
    (void)raw;
#endif
}

static void PrintFixedValue(const char *name, float value, const char *unit)
{
    int32_t scaled = (int32_t)(value * 100.0f);

    if (scaled < 0)
    {
        printf("%s=-%ld.%02ld %s", name,
               (long)((-scaled) / 100),
               (long)((-scaled) % 100),
               unit);
    }
    else
    {
        printf("%s=%ld.%02ld %s", name,
               (long)(scaled / 100),
               (long)(scaled % 100),
               unit);
    }
}

static void LogTapeInitial(uint8_t sensorNumber, uint8_t raw, uint8_t onTape)
{
#if defined(DEBUG) || defined(ROBOT_DEBUG)
    printf("[TAPE] init sensor%u raw=%u line=%s onTape=%u\r\n",
           (unsigned int)sensorNumber,
           (unsigned int)raw,
           raw ? "HIGH" : "LOW",
           (unsigned int)onTape);
#else
    (void)sensorNumber;
    (void)raw;
    (void)onTape;
#endif
}

static void LogTapeChange(uint8_t sensorNumber, uint8_t raw, uint8_t onTape)
{
#if ROBOT_REALTIME_TRACE
    printf("[TAPE] sensor%u raw=%u line=%s onTape=%u\r\n",
           (unsigned int)sensorNumber,
           (unsigned int)raw,
           raw ? "HIGH" : "LOW",
           (unsigned int)onTape);
#else
    (void)sensorNumber;
    (void)raw;
    (void)onTape;
#endif
}

static void LogTapeMask(uint8_t tapeMask, uint8_t changedMask)
{
#if (defined(DEBUG) || defined(ROBOT_DEBUG)) && ROBOT_LOG_TAPE
    printf("[TAPE] change mask=0x%02X changed=0x%02X "
           "s1=%s s2=%s s3=%s s4=%s s5=%s\r\n",
           (unsigned int)tapeMask,
           (unsigned int)changedMask,
           (tapeMask & TAPE_SENSOR_1_MASK) ? "ON " : "off",
           (tapeMask & TAPE_SENSOR_2_MASK) ? "ON " : "off",
           (tapeMask & TAPE_SENSOR_3_MASK) ? "ON " : "off",
           (tapeMask & TAPE_SENSOR_4_MASK) ? "ON " : "off",
           (tapeMask & TAPE_SENSOR_5_MASK) ? "ON " : "off");
#else
    (void) tapeMask;
    (void) changedMask;
#endif
}

static void LogTapeSample(const uint8_t raw[5], const uint8_t onTape[5])
{
#if ROBOT_REALTIME_TRACE
    printf("[TAPE] sample raw=%u/%u/%u/%u/%u on=%u/%u/%u/%u/%u\r\n",
           (unsigned int)raw[0],
           (unsigned int)raw[1],
           (unsigned int)raw[2],
           (unsigned int)raw[3],
           (unsigned int)raw[4],
           (unsigned int)onTape[0],
           (unsigned int)onTape[1],
           (unsigned int)onTape[2],
           (unsigned int)onTape[3],
           (unsigned int)onTape[4]);
#else
    (void)raw;
    (void)onTape;
#endif
}

static uint8_t TapeMaskFromReadings(const uint8_t onTape[5])
{
    uint8_t mask = 0u;
    uint8_t i;

    for (i = 0u; i < 5u; i++)
    {
        if (onTape[i] == TRUE)
        {
            mask |= (uint8_t)(1u << i);
        }
    }

    return mask;
}

static uint8_t BumpMaskFromReadings(const uint8_t bumped[4])
{
    uint8_t mask = 0u;
    uint8_t i;

    for (i = 0u; i < 4u; i++)
    {
        if (bumped[i] == TRUE)
        {
            mask |= (uint8_t)(1u << i);
        }
    }

    return mask;
}

static void LogBumpInitial(uint8_t sensorNumber, uint8_t raw, uint8_t bumped)
{
#if defined(DEBUG) || defined(ROBOT_DEBUG)
    printf("[BUMP] init sensor%u raw=%u line=%s bumped=%u\r\n",
           (unsigned int)sensorNumber,
           (unsigned int)raw,
           raw ? "HIGH" : "LOW",
           (unsigned int)bumped);
#else
    (void)sensorNumber;
    (void)raw;
    (void)bumped;
#endif
}

static void LogBumpChange(uint8_t sensorNumber, uint8_t raw, uint8_t bumped)
{
#if ROBOT_REALTIME_TRACE
    printf("[BUMP] sensor%u raw=%u line=%s bumped=%u\r\n",
           (unsigned int)sensorNumber,
           (unsigned int)raw,
           raw ? "HIGH" : "LOW",
           (unsigned int)bumped);
#else
    (void)sensorNumber;
    (void)raw;
    (void)bumped;
#endif
}

static void LogBumpMask(uint8_t bumpMask, uint8_t changedMask)
{
#if (defined(DEBUG) || defined(ROBOT_DEBUG)) && ROBOT_LOG_BUMP
    printf("[BUMP] mask=0x%02X changed=0x%02X -> %s\r\n",
           (unsigned int)bumpMask,
           (unsigned int)changedMask,
           EventNames[BumpChangedEvent]);
#else
    (void)bumpMask;
    (void)changedMask;
#endif
}
