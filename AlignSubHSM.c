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

/* Tape align (rewritten again to catch corner cases):
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
 *  GYRO mode (ShootingSubHSM and gyro nav states) is unchanged and driven
 *  continuously by AlignSubHSM_UpdateControl(). */
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
static float xRef = 0.0f;
static float yRef = 0.0f;
static uint8_t headingAlignedSamples = 0u;
/* Current search-run length (ms); grows by ALIGN_TIMER_STEP_MS each reversal. */
static uint16_t alignTimerMs = ALIGN_TIMER_START_MS;
/* Tape sensor mask sampled when align started, used to detect a corner-sensor
 * rising edge between entry and exit. */
static uint8_t tapeEntryMask = 0u;
/* Corner sensor for the active axis/boundary (2 horizontal, 4 top, 3 bottom). */
static uint8_t cornerSensor = 2u;
/* Latched TRUE once the corner sensor has been seen on during the primary run. */
static uint8_t cornerSeen = FALSE;
/* Jitter alternates a drive pulse and an active-brake pulse. */
static uint8_t jitterBraking = FALSE;
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
static uint8_t CornerIsOn(void);
static uint8_t CenterTapeIsOn(void);
static uint8_t Tape5Edge(ES_Event event, uint8_t wantOn);
static uint8_t CornerSensorForAlign(void);
static uint8_t TapeHeadingIsStraight(void);
static void IssueTapeHeadingTurn(void);
static void IssueTapeSearchMotion(uint8_t primary);
static void IssueTapeJitterMotion(void);
static void EnterTapeSearch(uint8_t primary);
static void CompleteTapeRealign(ES_Event *event);
static void PrintFixedDeg(float value);
static void LogAlignAction(const char *action, float headingErrDeg);

uint8_t InitAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches)
{
    return InitGyroAlignSubHSM(axis, xRefInches, yRefInches);
}

uint8_t InitGyroAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches)
{
    return InitAlignCommon(ALIGN_MODE_GYRO, axis, BOUNDARY_TOP, xRefInches, yRefInches);
}

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
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            AlignSubHSM_UpdateControl();
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

uint8_t AlignSubHSM_IsTapeStage(void)
{
    return ((CurrentState == TapeHeadingAlignState) ||
            (CurrentState == TapeSearchPrimaryState) ||
            (CurrentState == TapeSearchSecondaryState) ||
            (CurrentState == TapeJitterState)) ? TRUE : FALSE;
}

void AlignSubHSM_UpdateControl(void)
{
    float error;

    /* Only GYRO mode is driven continuously here. Tape align is timer-driven
     * inside RunAlignSubHSM (search runs and jitter pulses). */
    if (CurrentState != GyroHeadingAlignState) {
        return;
    }

    error = RobotIMU_GetHeadingErrorToRefDeg();
    if (AbsFloat(error) <= HEADING_THRESHOLD_DEG) {
        RobotMotion_Stop();
        LogAlignAction("gyro-centered-stop", error);
    } else if (error > 0.0f) {
        RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
        LogAlignAction("gyro-turn-left", error);
    } else {
        RobotMotion_TurnRightAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
        LogAlignAction("gyro-turn-right", error);
    }
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
    /* Snapshot the tape sensors so the corner rising-edge can be detected when
     * align finishes. */
    tapeEntryMask = LiveTapeMask();
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
