#include "AlignSubHSM.h"

#include "ES_Framework.h"
#include "ES_Timers.h"
#include "RobotDebug.h"
#include "RobotHSM.h"
#include "RobotIMU.h"
#include "RobotMotion.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"
#include "RobotSensors.h"

#include <stdio.h>

#if (defined(DEBUG) || defined(ROBOT_DEBUG)) && ROBOT_LOG_ALIGN
#define ALIGN_TRACE(...) printf(__VA_ARGS__)
#define ALIGN_LOG_ENABLED 1
#else
#define ALIGN_TRACE(...) ((void) 0)
#define ALIGN_LOG_ENABLED 0
#endif

/* DEPRECATED: TAPE ALIGN MODE IS NO LONGER USED.
 *
 * The NavigateToISZ rework aligns purely with the gyro (GYRO mode below) back
 * to a single heading latched on entry. The tape-align state machine
 * (TapeHeadingAlignState / TapeSearchPrimaryState / TapeSearchSecondaryState /
 * TapeJitterState), InitTapeAlignSubHSM(), and their helpers are retained for
 * reference but are now dead code -- nothing calls InitTapeAlignSubHSM().
 *
 * Tape align (rewritten again to catch corner cases):
 *
 *  Entered when center sensor 5 leaves the line. The robot runs a widening
 *  back-and-forth sweep along one axis to put sensor 5 back on the line, and
 *  also watches for the "corner" sensor so it can tell NavigateToISZ when the
 *  perpendicular line has been reached.
 *
 *  Entry first squares the heading: TapeHeadingAlignState jitter-turns about
 *  center (left for positive error, right for negative) in 100 ms drive /
 *  100 ms active-brake pulses until |error| <= TAPE_ALIGN_HEADING_STRAIGHT_DEG.
 *  The heading is measured against the latched global reference and is NEVER
 *  zeroed here. Once straight, align_timer is reset to 100 ms and control drops
 *  into the movement-axis branches (or completes if tape 5 is already on).
 *
 *  Per axis / boundary the directions and corner sensor are:
 *      horizontal      : primary reverse,      secondary forward,      corner 2
 *      vertical TOP     : primary strafe right, secondary strafe left, corner 4
 *      vertical BOTTOM  : primary strafe left,  secondary strafe right, corner 3
 *
 *  PRIMARY (run alignTimerMs, then += STEP):
 *      corner-seen AND tape 5 on  -> Realigned (+ corner edge)
 *      tape 5 OFF edge            -> Jitter (pulse the secondary direction)
 *      timeout                    -> Secondary
 *  SECONDARY (run alignTimerMs, then += STEP):
 *      tape 5 ON edge             -> Realigned (+ corner edge)
 *      timeout                    -> Primary
 *  JITTER (secondary direction, 100 ms drive / 100 ms active-brake pulses):
 *      tape 5 ON edge             -> Realigned (+ corner edge)
 *
 *  Corner edge: events only fire on tape transitions, so a corner sensor that
 *  rises while align owns the drive would never re-notify the nav state. We
 *  sample the corner sensor on align entry and again on align exit; if it was
 *  off on entry and on at exit we synthesize a TapeChangedEvent rising edge and
 *  post it to the top so the returned-to nav state can switch strafe -> reverse.
 *
 *  GYRO mode defers exact sensor events it receives while turning and replays
 *  those same events on exit, plus a live tape/bump reassertion, so caller
 *  states do not miss edges that occurred while align owned the drive. */
typedef enum {
    InitPAlignState,
    GyroHeadingAlignState,
    TapeHeadingAlignState,
    TapeSearchPrimaryState,
    TapeSearchSecondaryState,
    TapeJitterState,
} AlignState_t;

static const char *StateNames[] = {
    "InitPAlignState",
    "GyroHeadingAlignState",
    "TapeHeadingAlignState",
    "TapeSearchPrimaryState",
    "TapeSearchSecondaryState",
    "TapeJitterState",
};

static AlignState_t CurrentState = InitPAlignState;
static AlignMode_t alignMode = ALIGN_MODE_GYRO;
static MovementAxis_t alignAxis = MOVEMENT_AXIS_HORIZONTAL;
static BoundaryChoice_t alignBoundary = BOUNDARY_TOP;
/* Pivot used by the granular gyro correction turns (set per align so we can
 * rotate about the tape sensor the caller is tracking). */
static TurnPivot_t gyroPivot = TURN_PIVOT_CENTER;
static float xRef = 0.0f;
static float yRef = 0.0f;
static uint8_t headingAlignedSamples = 0u;
/* Current search-run length (ms); grows by ALIGN_TIMER_STEP_MS each reversal. */
static uint16_t alignTimerMs = ALIGN_TIMER_START_MS;
/* Tape sensor mask sampled when align started, used to detect a corner-sensor
 * rising edge between entry and exit. */
static uint8_t tapeEntryMask = 0u;
#define GYRO_DEFERRED_EVENT_CAPACITY 8u
static ES_Event gyroDeferredEvents[GYRO_DEFERRED_EVENT_CAPACITY];
static uint8_t gyroDeferredEventCount = 0u;
/* Corner sensor for the active axis/boundary (2 horizontal, 4 top, 3 bottom). */
static uint8_t cornerSensor = 2u;
/* Latched TRUE once the corner sensor has been seen on during the primary run. */
static uint8_t cornerSeen = FALSE;
/* Jitter alternates a drive pulse and an active-brake pulse. */
static uint8_t jitterBraking = FALSE;
/* Gyro align pulse counter (watchdog against infinite pulse loops). */
static uint8_t gyroAlignPulseCount = 0u;
#if ALIGN_LOG_ENABLED
/* Last commanded action we logged, so [ALIGN] prints only on a change. Stable
 * string literals, so pointer comparison is enough. */
static const char *lastAlignAction = "none";
#endif

static uint8_t InitAlignCommon(AlignMode_t mode, MovementAxis_t axis,
        BoundaryChoice_t boundary, float xRefInches, float yRefInches);
static float AbsFloat(float value);
static uint8_t StableCheck(uint8_t inThreshold, uint8_t *sampleCounter);
static uint8_t TapeMaskForSensor(uint8_t sensorNumber);
static uint8_t LiveTapeMask(void);
static uint8_t LiveBumpMask(void);
static uint8_t CornerIsOn(void);
static uint8_t CenterTapeIsOn(void);
static uint8_t Tape5Edge(ES_Event event, uint8_t wantOn);
static uint8_t CornerSensorForAlign(void);
static uint8_t GyroHeadingIsStraight(void);
static uint16_t GyroTurnPulseMs(void);
static void IssueGyroHeadingTurn(void);
static void ClearGyroDeferredEvents(void);
static void RecordGyroDeferredEvent(ES_Event event);
static void FlushGyroDeferredEvents(void);
static void ReassertGyroSensorEvents(void);
static uint8_t TapeHeadingIsStraight(void);
static void IssueTapeHeadingTurn(void);
static void IssueTapeSearchMotion(uint8_t primary);
static void IssueTapeJitterMotion(void);
static void EnterTapeSearch(uint8_t primary);
static void CompleteGyroRealign(ES_Event *event);
static void CompleteTapeRealign(ES_Event *event);
static void PrintFixedDeg(float value);
static void LogAlignAction(const char *action, float headingErrDeg);

uint8_t InitAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches)
{
    return InitGyroAlignSubHSM(axis, TURN_PIVOT_CENTER, xRefInches, yRefInches);
}

uint8_t InitGyroAlignSubHSM(MovementAxis_t axis, TurnPivot_t pivot,
        float xRefInches, float yRefInches)
{
    /* Active-brake any prior drive command before the align takes over. */
    RobotMotion_Stop();
    gyroPivot = pivot;
    return InitAlignCommon(ALIGN_MODE_GYRO, axis, BOUNDARY_TOP, xRefInches, yRefInches);
}

/* DEPRECATED: tape align is no longer used (gyro-only NavigateToISZ rework). */
uint8_t InitTapeAlignSubHSM(MovementAxis_t axis, BoundaryChoice_t boundary,
        float xRefInches, float yRefInches)
{
    return InitAlignCommon(ALIGN_MODE_TAPE, axis, boundary, xRefInches, yRefInches);
}

ES_Event RunAlignSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE;
    AlignState_t nextState = CurrentState;

    ES_Tattle();
    ROBOT_DEBUG_STATE("Align", StateNames[CurrentState], ThisEvent);

    switch (CurrentState) {
    case InitPAlignState:
        if (ThisEvent.EventType == ES_INIT) {
            nextState = (alignMode == ALIGN_MODE_TAPE) ?
                    TapeHeadingAlignState : GyroHeadingAlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case GyroHeadingAlignState:
        /* Granular correction: short turn pulse about gyroPivot, then an active
         * brake/settle, repeated until the heading error is within
         * GYRO_ALIGN_STRAIGHT_DEG. jitterBraking marks the brake half of the
         * cycle. Completion is signalled by returning RealignedEvent up to the
         * caller (NavigateToISZ / ShootingSubHSM) on a settle timeout. */
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotIMU_UpdateGyroHeading();
            if (GyroHeadingIsStraight() == TRUE) {
                /* Already squared: brake and let the next settle complete (the
                 * entry return value is discarded by the transition code). */
                RobotMotion_Stop();
                jitterBraking = TRUE;
                ES_Timer_InitTimer(NAV_SETTLE_TIMER, GYRO_ALIGN_BRAKE_MS);
            } else {
                jitterBraking = FALSE;
                gyroAlignPulseCount = 0u;
                IssueGyroHeadingTurn();
                ES_Timer_InitTimer(NAV_SETTLE_TIMER, GyroTurnPulseMs());
            }
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                if (jitterBraking == FALSE) {
                    /* End the turn pulse with an active brake, then rest so the
                     * heading reading settles before we re-evaluate it. */
                    RobotMotion_Stop();
                    RobotIMU_UpdateGyroHeading();
                    jitterBraking = TRUE;
                    ES_Timer_InitTimer(NAV_SETTLE_TIMER, GYRO_ALIGN_BRAKE_MS);
                    ThisEvent.EventType = ES_NO_EVENT;
                } else {
                    jitterBraking = FALSE;
                    RobotIMU_UpdateGyroHeading();
                    if ((GyroHeadingIsStraight() == TRUE) ||
                            (gyroAlignPulseCount >= GYRO_ALIGN_MAX_PULSES)) {
                        CompleteGyroRealign(&ThisEvent);
#if ALIGN_LOG_ENABLED
                        if (gyroAlignPulseCount >= GYRO_ALIGN_MAX_PULSES) {
                            printf("[ALIGN] gyro-align max pulses (%u), exiting\r\n",
                                    (unsigned int) GYRO_ALIGN_MAX_PULSES);
                        }
#endif
                    } else {
                        IssueGyroHeadingTurn();
                        ES_Timer_InitTimer(NAV_SETTLE_TIMER, GyroTurnPulseMs());
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                }
            }
            break;
        case BeaconADCIncreaseEvent:
        case MaxSignalFoundEvent:
            RecordGyroDeferredEvent(ThisEvent);
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case TapeChangedEvent:
        case BumpChangedEvent:
            RecordGyroDeferredEvent(ThisEvent);
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case PositionRealignedEvent:
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case RealignedEvent:
            RobotMotion_Stop();
            ThisEvent.EventType = RealignedEvent;
            break;
        default:
            break;
        }
        break;

    case TapeHeadingAlignState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            jitterBraking = FALSE;
            if (TapeHeadingIsStraight() == TRUE) {
                /* Heading already squared: start the sweep right away. */
                RobotMotion_Stop();
                alignTimerMs = ALIGN_TIMER_START_MS;
                nextState = TapeSearchPrimaryState;
                makeTransition = TRUE;
            } else {
                IssueTapeHeadingTurn();
                ES_Timer_InitTimer(NAV_SETTLE_TIMER, ALIGN_MOTION_PULSE_MS);
            }
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case TapeChangedEvent:
            /* Heading squaring ignores tape edges; the sweep handles tape 5. */
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                if (jitterBraking == FALSE) {
                    /* End the turn pulse with an active brake, then rest so the
                     * heading reading settles before we re-evaluate it. */
                    RobotMotion_Stop();
                    jitterBraking = TRUE;
                    ES_Timer_InitTimer(NAV_SETTLE_TIMER, ALIGN_MOTION_PULSE_MS);
                    ThisEvent.EventType = ES_NO_EVENT;
                } else {
                    jitterBraking = FALSE;
                    if (TapeHeadingIsStraight() == TRUE) {
                        if (CenterTapeIsOn() == TRUE) {
                            /* Squared and already on the line: nothing to sweep. */
                            CompleteTapeRealign(&ThisEvent);
                        } else {
                            alignTimerMs = ALIGN_TIMER_START_MS;
                            nextState = TapeSearchPrimaryState;
                            makeTransition = TRUE;
                            ThisEvent.EventType = ES_NO_EVENT;
                        }
                    } else {
                        IssueTapeHeadingTurn();
                        ES_Timer_InitTimer(NAV_SETTLE_TIMER, ALIGN_MOTION_PULSE_MS);
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                }
            }
            break;
        case RealignedEvent:
            RobotMotion_Stop();
            ThisEvent.EventType = RealignedEvent;
            break;
        default:
            break;
        }
        break;

    case TapeSearchPrimaryState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            if (CornerIsOn() == TRUE) {
                cornerSeen = TRUE;
            }
            EnterTapeSearch(TRUE);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case TapeChangedEvent: {
            uint8_t currentMask = TAPE_EVENT_CURRENT_MASK(ThisEvent.EventParam);
            if ((currentMask & TapeMaskForSensor(cornerSensor)) != 0u) {
                cornerSeen = TRUE;
            }
            if ((cornerSeen == TRUE) &&
                    ((currentMask & TAPE_SENSOR_5_MASK) != 0u)) {
                CompleteTapeRealign(&ThisEvent);
            } else if (Tape5Edge(ThisEvent, FALSE) == TRUE) {
                nextState = TapeJitterState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        }
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = TapeSearchSecondaryState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case RealignedEvent:
            RobotMotion_Stop();
            ThisEvent.EventType = RealignedEvent;
            break;
        default:
            break;
        }
        break;

    case TapeSearchSecondaryState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            EnterTapeSearch(FALSE);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case TapeChangedEvent:
            if (Tape5Edge(ThisEvent, TRUE) == TRUE) {
                CompleteTapeRealign(&ThisEvent);
            } else {
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = TapeSearchPrimaryState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case RealignedEvent:
            RobotMotion_Stop();
            ThisEvent.EventType = RealignedEvent;
            break;
        default:
            break;
        }
        break;

    case TapeJitterState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            jitterBraking = FALSE;
            IssueTapeJitterMotion();
            ES_Timer_InitTimer(NAV_SETTLE_TIMER, ALIGN_MOTION_PULSE_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case TapeChangedEvent:
            if (Tape5Edge(ThisEvent, TRUE) == TRUE) {
                CompleteTapeRealign(&ThisEvent);
            } else {
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                if (jitterBraking == FALSE) {
                    /* End the drive pulse with an active brake, then rest. */
                    RobotMotion_Stop();
                    jitterBraking = TRUE;
                } else {
                    /* Brake pulse done: drive the next jitter step. */
                    jitterBraking = FALSE;
                    IssueTapeJitterMotion();
                }
                ES_Timer_InitTimer(NAV_SETTLE_TIMER, ALIGN_MOTION_PULSE_MS);
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case RealignedEvent:
            RobotMotion_Stop();
            ThisEvent.EventType = RealignedEvent;
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }

    if (makeTransition == TRUE) {
        RunAlignSubHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunAlignSubHSM(ENTRY_EVENT);
        RobotMotion_DebugPrintCurrentCommand("entry");
    }

    ES_Tail();
    return ThisEvent;
}

uint8_t AlignSubHSM_IsActive(void)
{
    return (CurrentState != InitPAlignState) ? TRUE : FALSE;
}

uint8_t AlignSubHSM_IsPositionStage(void)
{
    return FALSE;
}

uint8_t AlignSubHSM_IsHeadingStage(void)
{
    return (CurrentState == GyroHeadingAlignState) ? TRUE : FALSE;
}

/* DEPRECATED: tape align stages are unreachable in the gyro-only rework. */
uint8_t AlignSubHSM_IsTapeStage(void)
{
    return ((CurrentState == TapeHeadingAlignState) ||
            (CurrentState == TapeSearchPrimaryState) ||
            (CurrentState == TapeSearchSecondaryState) ||
            (CurrentState == TapeJitterState)) ? TRUE : FALSE;
}

/* DEPRECATED: gyro align used to be driven continuously from the event checker
 * (CheckAlignEvents) which overshot badly. Gyro align is now self-driven by the
 * granular turn/brake pulses inside GyroHeadingAlignState, so this is a no-op. */
void AlignSubHSM_UpdateControl(void)
{
}

uint8_t AlignSubHSM_IsPositionAligned(void)
{
    return FALSE;
}

uint8_t AlignSubHSM_IsHeadingAligned(void)
{
    if (CurrentState != GyroHeadingAlignState) {
        headingAlignedSamples = 0u;
        return FALSE;
    }
    return StableCheck(AbsFloat(RobotIMU_GetHeadingErrorToRefDeg()) <= HEADING_THRESHOLD_DEG,
            &headingAlignedSamples);
}

const char *AlignSubHSM_GetStateName(void)
{
    return StateNames[CurrentState];
}

MovementAxis_t AlignSubHSM_GetAxis(void)
{
    return alignAxis;
}

AlignMode_t AlignSubHSM_GetMode(void)
{
    return alignMode;
}

float AlignSubHSM_GetXRef(void)
{
    return xRef;
}

float AlignSubHSM_GetYRef(void)
{
    return yRef;
}

float AlignSubHSM_GetPositionError(void)
{
    if (alignAxis == MOVEMENT_AXIS_HORIZONTAL) {
        return yRef - RobotIMU_GetYInches();
    }
    return xRef - RobotIMU_GetXInches();
}

static uint8_t InitAlignCommon(AlignMode_t mode, MovementAxis_t axis,
        BoundaryChoice_t boundary, float xRefInches, float yRefInches)
{
    ES_Event returnEvent;

    alignMode = mode;
    alignAxis = axis;
    alignBoundary = boundary;
    xRef = xRefInches;
    yRef = yRefInches;
    CurrentState = InitPAlignState;
    headingAlignedSamples = 0u;
    alignTimerMs = ALIGN_TIMER_START_MS;
    cornerSensor = CornerSensorForAlign();
    cornerSeen = FALSE;
    jitterBraking = FALSE;
    gyroAlignPulseCount = 0u;
    tapeEntryMask = (mode == ALIGN_MODE_TAPE) ? LiveTapeMask() : 0u;
    ClearGyroDeferredEvents();
#if ALIGN_LOG_ENABLED
    lastAlignAction = "none";
    if (mode == ALIGN_MODE_TAPE) {
        printf("[ALIGN] tape-align start axis=%s boundary=%s corner=tape%u entryMask=0x%02X\r\n",
                (axis == MOVEMENT_AXIS_VERTICAL) ? "V" : "H",
                (boundary == BOUNDARY_TOP) ? "TOP" : "BOTTOM",
                (unsigned int) cornerSensor,
                (unsigned int) tapeEntryMask);
    }
#endif

    returnEvent = RunAlignSubHSM(INIT_EVENT);
    return (returnEvent.EventType == ES_NO_EVENT) ? TRUE : FALSE;
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t StableCheck(uint8_t inThreshold, uint8_t *sampleCounter)
{
    if (inThreshold == FALSE) {
        *sampleCounter = 0u;
        return FALSE;
    }

    if (*sampleCounter < ALIGN_STABLE_SAMPLE_COUNT) {
        (*sampleCounter)++;
    }

    return (*sampleCounter >= ALIGN_STABLE_SAMPLE_COUNT) ? TRUE : FALSE;
}

static uint8_t TapeMaskForSensor(uint8_t sensorNumber)
{
    if ((sensorNumber < 1u) || (sensorNumber > 5u)) {
        return 0u;
    }
    return (uint8_t) (1u << (sensorNumber - 1u));
}

static uint8_t LiveTapeMask(void)
{
    uint8_t mask = 0u;
    uint8_t sensor;

    for (sensor = 1u; sensor <= 5u; sensor++) {
        if ((RobotPlugPlay_IsTapeEnabled(sensor) == TRUE) &&
                (RobotSensors_IsTapeOn((TapeSensor_t) sensor) == TRUE)) {
            mask |= TapeMaskForSensor(sensor);
        }
    }
    return mask;
}

static uint8_t LiveBumpMask(void)
{
    uint8_t mask = 0u;
    uint8_t sensor;

    for (sensor = 1u; sensor <= 4u; sensor++) {
        if ((RobotPlugPlay_IsBumpEnabled(sensor) == TRUE) &&
                (RobotSensors_IsBumpOn((BumpSensor_t) sensor) == TRUE)) {
            mask |= (uint8_t) (1u << (sensor - 1u));
        }
    }
    return mask;
}

static uint8_t CornerIsOn(void)
{
    if (RobotPlugPlay_IsTapeEnabled(cornerSensor) == FALSE) {
        return FALSE;
    }
    return (RobotSensors_IsTapeOn((TapeSensor_t) cornerSensor) == TRUE) ? TRUE : FALSE;
}

static uint8_t CenterTapeIsOn(void)
{
    return (RobotSensors_IsTapeOn(TAPE_SENSOR_5) == TRUE) ? TRUE : FALSE;
}

static uint8_t GyroHeadingIsStraight(void)
{
    return (AbsFloat(RobotIMU_GetGyroHeadingErrorToRefDeg()) <=
            GYRO_ALIGN_STRAIGHT_DEG) ? TRUE : FALSE;
}

/* Turn pulse length grows with |heading error| between MIN and MAX so tiny
 * errors get short nudges and larger misalignments get a bit more rotation per
 * step without reverting to a continuous turn. */
static uint16_t GyroTurnPulseMs(void)
{
    float err = AbsFloat(RobotIMU_GetGyroHeadingErrorToRefDeg());
    float span;
    float scaled;
    uint16_t extra;

    if (err <= GYRO_ALIGN_STRAIGHT_DEG) {
        return GYRO_ALIGN_TURN_PULSE_MIN_MS;
    }

    span = HEADING_THRESHOLD_DEG - GYRO_ALIGN_STRAIGHT_DEG;
    if (span < 0.5f) {
        span = 0.5f;
    }
    scaled = (err - GYRO_ALIGN_STRAIGHT_DEG) / span;
    if (scaled > 1.0f) {
        scaled = 1.0f;
    }

    extra = (uint16_t) (scaled *
            (float) (GYRO_ALIGN_TURN_PULSE_MAX_MS - GYRO_ALIGN_TURN_PULSE_MIN_MS) +
            0.5f);
    return (uint16_t) (GYRO_ALIGN_TURN_PULSE_MIN_MS + extra);
}

/* One granular heading-correction pulse about gyroPivot. Positive error
 * (heading right of the saved reference) turns left, negative turns right.
 * Heading is measured against the latched reference and never zeroed here. */
static void IssueGyroHeadingTurn(void)
{
    float error;

    RobotIMU_UpdateGyroHeading();
    error = RobotIMU_GetGyroHeadingErrorToRefDeg();

    gyroAlignPulseCount++;
    if (error > 0.0f) {
        RobotMotion_TurnLeftAbout(gyroPivot, ALIGN_SPEED_IPS);
        LogAlignAction("gyro-pulse-left", error);
    } else {
        RobotMotion_TurnRightAbout(gyroPivot, ALIGN_SPEED_IPS);
        LogAlignAction("gyro-pulse-right", error);
    }
#if ALIGN_LOG_ENABLED
    printf("[ALIGN] gyro pulse %ums (pulse %u)\r\n",
            (unsigned int) GyroTurnPulseMs(),
            (unsigned int) gyroAlignPulseCount);
#endif
}

static void ClearGyroDeferredEvents(void)
{
    gyroDeferredEventCount = 0u;
}

static void RecordGyroDeferredEvent(ES_Event event)
{
    if (gyroDeferredEventCount >= GYRO_DEFERRED_EVENT_CAPACITY) {
        return;
    }

    gyroDeferredEvents[gyroDeferredEventCount] = event;
    gyroDeferredEventCount++;
}

static void FlushGyroDeferredEvents(void)
{
    uint8_t i;

    for (i = 0u; i < gyroDeferredEventCount; i++) {
        PostRobotHSM(gyroDeferredEvents[i]);
#if ALIGN_LOG_ENABLED
        printf("[ALIGN] replay %s param=0x%X\r\n",
                EventNames[gyroDeferredEvents[i].EventType],
                (unsigned int) gyroDeferredEvents[i].EventParam);
#endif
    }

    ClearGyroDeferredEvents();
}

static void ReassertGyroSensorEvents(void)
{
    ES_Event event;
    uint8_t tapeMask = LiveTapeMask();
    uint8_t bumpMask = LiveBumpMask();

    event.EventType = TapeChangedEvent;
    event.EventParam = TAPE_EVENT_MAKE_PARAM(tapeMask, TAPE_SENSOR_ALL_MASK);
    PostRobotHSM(event);

    event.EventType = BumpChangedEvent;
    event.EventParam = BUMP_EVENT_MAKE_PARAM(bumpMask, BUMP_SENSOR_ALL_MASK);
    PostRobotHSM(event);
#if ALIGN_LOG_ENABLED
    printf("[ALIGN] reassert tape current=0x%02X bump current=0x%02X\r\n",
            (unsigned int) tapeMask,
            (unsigned int) bumpMask);
#endif
}

static uint8_t TapeHeadingIsStraight(void)
{
    return (AbsFloat(RobotIMU_GetHeadingErrorToRefDeg()) <=
            TAPE_ALIGN_HEADING_STRAIGHT_DEG) ? TRUE : FALSE;
}

/* One heading-squaring turn pulse about center. Positive error (heading right
 * of the reference) turns left, negative turns right. Heading is taken against
 * the latched global reference and is never zeroed. */
static void IssueTapeHeadingTurn(void)
{
    float error = RobotIMU_GetHeadingErrorToRefDeg();

    if (error > 0.0f) {
        RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
        LogAlignAction("tape-head-jitter-left", error);
    } else {
        RobotMotion_TurnRightAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
        LogAlignAction("tape-head-jitter-right", error);
    }
}

/* TRUE when the event carries a tape-5 transition matching wantOn (1 = a fresh
 * tape-5-ON edge, 0 = a fresh tape-5-OFF edge). */
static uint8_t Tape5Edge(ES_Event event, uint8_t wantOn)
{
    uint8_t currentMask;
    uint8_t changedMask;

    if (event.EventType != TapeChangedEvent) {
        return FALSE;
    }
    currentMask = TAPE_EVENT_CURRENT_MASK(event.EventParam);
    changedMask = TAPE_EVENT_CHANGED_MASK(event.EventParam);
    if ((changedMask & TAPE_SENSOR_5_MASK) == 0u) {
        return FALSE;
    }
    if (wantOn == TRUE) {
        return ((currentMask & TAPE_SENSOR_5_MASK) != 0u) ? TRUE : FALSE;
    }
    return ((currentMask & TAPE_SENSOR_5_MASK) == 0u) ? TRUE : FALSE;
}

static uint8_t CornerSensorForAlign(void)
{
    if (alignAxis == MOVEMENT_AXIS_HORIZONTAL) {
        return 2u;
    }
    return (alignBoundary == BOUNDARY_TOP) ? 4u : 3u;
}

/* Primary search direction (and its opposite for secondary/jitter):
 *      horizontal     : primary reverse,      secondary forward
 *      vertical TOP    : primary strafe right, secondary strafe left
 *      vertical BOTTOM : primary strafe left,  secondary strafe right */
static void IssueTapeSearchMotion(uint8_t primary)
{
    if (alignAxis == MOVEMENT_AXIS_HORIZONTAL) {
        if (primary == TRUE) {
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
            LogAlignAction("tape-search-reverse", RobotIMU_GetHeadingErrorToRefDeg());
        } else {
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            LogAlignAction("tape-search-forward", RobotIMU_GetHeadingErrorToRefDeg());
        }
    } else {
        uint8_t primaryGoesRight = (alignBoundary == BOUNDARY_TOP) ? TRUE : FALSE;
        uint8_t goRight = (primary == TRUE) ? primaryGoesRight
                : (uint8_t) (primaryGoesRight == FALSE);
        if (goRight == TRUE) {
            RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
            LogAlignAction("tape-search-strafe-right", RobotIMU_GetHeadingErrorToRefDeg());
        } else {
            RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            LogAlignAction("tape-search-strafe-left", RobotIMU_GetHeadingErrorToRefDeg());
        }
    }
}

/* Jitter drives the secondary (opposite of primary) direction. */
static void IssueTapeJitterMotion(void)
{
    if (alignAxis == MOVEMENT_AXIS_HORIZONTAL) {
        RobotMotion_Forward(MOTOR_SPEED_IPS);
        LogAlignAction("tape-jitter-forward", RobotIMU_GetHeadingErrorToRefDeg());
    } else if (alignBoundary == BOUNDARY_TOP) {
        RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
        LogAlignAction("tape-jitter-strafe-left", RobotIMU_GetHeadingErrorToRefDeg());
    } else {
        RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
        LogAlignAction("tape-jitter-strafe-right", RobotIMU_GetHeadingErrorToRefDeg());
    }
}

static void EnterTapeSearch(uint8_t primary)
{
    IssueTapeSearchMotion(primary);
    ES_Timer_InitTimer(NAV_SETTLE_TIMER, alignTimerMs);
    alignTimerMs = (uint16_t) (alignTimerMs + ALIGN_TIMER_STEP_MS);
}

static void CompleteGyroRealign(ES_Event *event)
{
    RobotMotion_Stop();
    ReassertGyroSensorEvents();
    FlushGyroDeferredEvents();

    event->EventType = RealignedEvent;
    event->EventParam = ALIGN_REALIGNED_SOURCE_SENSOR;
}

/* Stop, mark the event as Realigned, and synthesize the corner-sensor rising
 * edge for the nav state if the corner sensor rose between align entry and now.
 * The synthetic TapeChangedEvent is posted to the top so it is delivered after
 * nav has returned from AlignState to its strafe/drive state. */
static void CompleteTapeRealign(ES_Event *event)
{
    uint8_t cornerMask = TapeMaskForSensor(cornerSensor);
    uint8_t roseAtCorner = (((tapeEntryMask & cornerMask) == 0u) &&
            (CornerIsOn() == TRUE)) ? TRUE : FALSE;

    RobotMotion_Stop();

    if (roseAtCorner == TRUE) {
        ES_Event tapeEvent;
        uint8_t liveMask = (uint8_t) (LiveTapeMask() | cornerMask);

        tapeEvent.EventType = TapeChangedEvent;
        tapeEvent.EventParam = TAPE_EVENT_MAKE_PARAM(liveMask, cornerMask);
        PostRobotHSM(tapeEvent);
#if ALIGN_LOG_ENABLED
        printf("[ALIGN] corner rising edge tape%u synthesized (entry off -> exit on)\r\n",
                (unsigned int) cornerSensor);
#endif
    }

    event->EventType = RealignedEvent;
    event->EventParam = ALIGN_REALIGNED_SOURCE_SENSOR;
}

static void LogAlignAction(const char *action, float headingErrDeg)
{
#if ALIGN_LOG_ENABLED
    if (action != lastAlignAction) {
        printf("[ALIGN] %s headingErr=", action);
        PrintFixedDeg(headingErrDeg);
        printf("\r\n");
        lastAlignAction = action;
    }
#else
    (void) action;
    (void) headingErrDeg;
#endif
}

static void PrintFixedDeg(float value)
{
    int scaled;

    if (value < 0.0f) {
        printf("-");
        value = -value;
    }

    scaled = (int) ((value * 100.0f) + 0.5f);
    printf("%d.%02d deg", scaled / 100, scaled % 100);
}
