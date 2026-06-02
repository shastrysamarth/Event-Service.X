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

typedef enum {
    InitPAlignState,
    GyroHeadingAlignState,
    TapeWaitOffState,
    TapeTurnLeftState,
    TapeTurnRightState,
    /* Heading-straight branch: pivot left about center until an edge sensor
     * (3/4 horizontal, 1/2 vertical) finds the line, then translate so center
     * sensor 5 lands on the line. Replaces the old timed left/right sweep. */
    TapeStraightPivotState,
    TapeStraightTranslateState,
} AlignState_t;

/* Translate direction chosen once an edge sensor finds the line, used to bring
 * center sensor 5 onto the line in the heading-straight branch. */
typedef enum {
    STRAIGHT_TRANSLATE_NONE,
    STRAIGHT_TRANSLATE_FORWARD,
    STRAIGHT_TRANSLATE_REVERSE,
    STRAIGHT_TRANSLATE_STRAFE_LEFT,
    STRAIGHT_TRANSLATE_STRAFE_RIGHT,
} StraightTranslate_t;

/* Result of re-evaluating the heading-straight pivot from live tape readings. */
typedef enum {
    STRAIGHT_KEEP_PIVOT,
    STRAIGHT_GO_TRANSLATE,
    STRAIGHT_DONE,
} StraightDecision_t;

typedef struct {
    uint8_t sensorA;
    uint8_t sensorB;
    uint8_t pivotSensor;
    uint8_t realignMask;
} TapeAlignBranch_t;

static const char *StateNames[] = {
    "InitPAlignState",
    "GyroHeadingAlignState",
    "TapeWaitOffState",
    "TapeTurnLeftState",
    "TapeTurnRightState",
    "TapeStraightPivotState",
    "TapeStraightTranslateState",
};

static const TapeAlignBranch_t VerticalBranches[] = {
    {1u, 2u, 5u, TAPE_SENSOR_1_MASK | TAPE_SENSOR_2_MASK},
    {1u, 5u, 2u, TAPE_SENSOR_5_MASK},
    {2u, 5u, 1u, TAPE_SENSOR_5_MASK},
};

static const TapeAlignBranch_t HorizontalBranches[] = {
    {3u, 4u, 5u, TAPE_SENSOR_3_MASK | TAPE_SENSOR_4_MASK},
    {3u, 5u, 4u, TAPE_SENSOR_5_MASK},
    {4u, 5u, 3u, TAPE_SENSOR_5_MASK},
};

static AlignState_t CurrentState = InitPAlignState;
static AlignMode_t alignMode = ALIGN_MODE_GYRO;
static MovementAxis_t alignAxis = MOVEMENT_AXIS_HORIZONTAL;
static float xRef = 0.0f;
static float yRef = 0.0f;
static uint8_t headingAlignedSamples = 0u;
static uint8_t tapeOffFlags = 0u;
static TapeAlignBranch_t activeTapeBranch = {
    1u, 2u, 5u,
    TAPE_SENSOR_1_MASK | TAPE_SENSOR_2_MASK
};
static StraightTranslate_t straightTranslateDir = STRAIGHT_TRANSLATE_NONE;
#if ALIGN_LOG_ENABLED
/* Last commanded turn direction we logged, so [ALIGN] prints only on a change
 * (UpdateControl runs every checker cycle while aligning). Stable literals, so
 * pointer comparison is enough. */
static const char *lastAlignAction = "none";
#endif

static uint8_t InitAlignCommon(AlignMode_t mode, MovementAxis_t axis,
        float xRefInches, float yRefInches);
static float AbsFloat(float value);
static uint8_t StableCheck(uint8_t inThreshold, uint8_t *sampleCounter);
static uint8_t TapeMaskForSensor(uint8_t sensorNumber);
static uint8_t TapeOffMaskForBranch(const TapeAlignBranch_t *branch);
static const TapeAlignBranch_t *SelectReadyTapeBranch(void);
static uint8_t StartTapeBranch(const TapeAlignBranch_t *branch,
        AlignState_t *nextState);
static AlignState_t TapeTurnStateForHeading(void);
static void RefreshTapeOffFlagsFromHardware(void);
static uint8_t UpdateTapeFlagsFromChangedMask(uint8_t currentMask,
        uint8_t changedMask);
static uint8_t CompleteIfRealignedTapeOn(ES_Event *event);
static TurnPivot_t PivotForTapeSensor(uint8_t sensorNumber);
static void DriveTapeTurn(uint8_t turnLeft);
static StraightDecision_t EvaluateStraightPivot(void);
static void DriveStraightTranslate(void);
static void PrintFixedDeg(float value);
static void LogAlignAction(const char *action, float headingErrDeg);

uint8_t InitAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches)
{
    return InitGyroAlignSubHSM(axis, xRefInches, yRefInches);
}

uint8_t InitGyroAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches)
{
    return InitAlignCommon(ALIGN_MODE_GYRO, axis, xRefInches, yRefInches);
}

uint8_t InitTapeAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches)
{
    return InitAlignCommon(ALIGN_MODE_TAPE, axis, xRefInches, yRefInches);
}

ES_Event RunAlignSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE;
    AlignState_t nextState = CurrentState;
    const TapeAlignBranch_t *branch;

    ES_Tattle();
    ROBOT_DEBUG_STATE("Align", StateNames[CurrentState], ThisEvent);

    switch (CurrentState) {
    case InitPAlignState:
        if (ThisEvent.EventType == ES_INIT) {
            nextState = (alignMode == ALIGN_MODE_TAPE) ?
                    TapeWaitOffState : GyroHeadingAlignState;
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

    case TapeWaitOffState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RefreshTapeOffFlagsFromHardware();
            branch = SelectReadyTapeBranch();
            if (branch != (const TapeAlignBranch_t *) 0) {
                makeTransition = StartTapeBranch(branch, &nextState);
            } else {
                /* No sensor pair is off, so there is nothing this align can
                 * correct. Do NOT sit here with the drive stopped: report
                 * aligned immediately so the caller resumes its motion. */
                RobotMotion_Stop();
                ThisEvent.EventType = RealignedEvent;
                ThisEvent.EventParam = ALIGN_REALIGNED_SOURCE_SENSOR;
            }
            break;
        case TapeChangedEvent:
            if (UpdateTapeFlagsFromChangedMask(
                    TAPE_EVENT_CURRENT_MASK(ThisEvent.EventParam),
                    TAPE_EVENT_CHANGED_MASK(ThisEvent.EventParam)) == TRUE) {
                branch = SelectReadyTapeBranch();
                if (branch != (const TapeAlignBranch_t *) 0) {
                    makeTransition = StartTapeBranch(branch, &nextState);
                }
            }
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case TapeTurnLeftState:
    case TapeTurnRightState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            AlignSubHSM_UpdateControl();
            break;
        case RealignedEvent:
            RobotMotion_Stop();
            ThisEvent.EventType = RealignedEvent;
            break;
        default:
            /* CompleteIfRealignedTapeOn consumes TapeChangedEvent: it updates
             * the off-flags and either completes (RealignedEvent) or clears the
             * event. Non-tape events pass through unchanged. */
            (void) CompleteIfRealignedTapeOn(&ThisEvent);
            break;
        }
        break;

    case TapeStraightPivotState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            /* An edge sensor may already be on the line (e.g. the 3+5 / 4+5
             * off pairs leave the other edge sensor reading tape), so try to
             * pick a translate direction right away; otherwise pivot left. */
            if (EvaluateStraightPivot() == STRAIGHT_GO_TRANSLATE) {
                nextState = TapeStraightTranslateState;
                makeTransition = TRUE;
            } else {
                AlignSubHSM_UpdateControl();
            }
            break;
        case TapeChangedEvent: {
            StraightDecision_t decision = EvaluateStraightPivot();
            if (decision == STRAIGHT_GO_TRANSLATE) {
                nextState = TapeStraightTranslateState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (decision == STRAIGHT_DONE) {
                /* Center sensor already found the line: nothing to translate. */
                RobotMotion_Stop();
                ThisEvent.EventType = RealignedEvent;
                ThisEvent.EventParam = ALIGN_REALIGNED_SOURCE_SENSOR;
            } else {
                AlignSubHSM_UpdateControl();
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        }
        case RealignedEvent:
            RobotMotion_Stop();
            ThisEvent.EventType = RealignedEvent;
            break;
        default:
            break;
        }
        break;

    case TapeStraightTranslateState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            AlignSubHSM_UpdateControl();
            break;
        case TapeChangedEvent:
            if (RobotSensors_IsTapeOn(TAPE_SENSOR_5) == TRUE) {
                RobotMotion_Stop();
                ThisEvent.EventType = RealignedEvent;
                ThisEvent.EventParam = ALIGN_REALIGNED_SOURCE_SENSOR;
            } else {
                AlignSubHSM_UpdateControl();
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
    return ((CurrentState == TapeWaitOffState) ||
            (CurrentState == TapeTurnLeftState) ||
            (CurrentState == TapeTurnRightState) ||
            (CurrentState == TapeStraightPivotState) ||
            (CurrentState == TapeStraightTranslateState)) ? TRUE : FALSE;
}

void AlignSubHSM_UpdateControl(void)
{
    float error;

    switch (CurrentState) {
    case GyroHeadingAlignState:
        error = RobotIMU_GetHeadingErrorToZeroDeg();
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
        break;
    case TapeTurnLeftState:
        DriveTapeTurn(TRUE);
        break;
    case TapeTurnRightState:
        DriveTapeTurn(FALSE);
        break;
    case TapeStraightPivotState:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
        LogAlignAction("straight-pivot-left", RobotIMU_GetHeadingErrorToZeroDeg());
        break;
    case TapeStraightTranslateState:
        DriveStraightTranslate();
        break;
    default:
        break;
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
    return StableCheck(AbsFloat(RobotIMU_GetHeadingErrorToZeroDeg()) <= HEADING_THRESHOLD_DEG,
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
        float xRefInches, float yRefInches)
{
    ES_Event returnEvent;

    alignMode = mode;
    alignAxis = axis;
    xRef = xRefInches;
    yRef = yRefInches;
    CurrentState = InitPAlignState;
    headingAlignedSamples = 0u;
    tapeOffFlags = 0u;
    straightTranslateDir = STRAIGHT_TRANSLATE_NONE;
#if ALIGN_LOG_ENABLED
    lastAlignAction = "none";
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

static uint8_t TapeOffMaskForBranch(const TapeAlignBranch_t *branch)
{
    return (uint8_t) (TapeMaskForSensor(branch->sensorA) |
            TapeMaskForSensor(branch->sensorB));
}

static const TapeAlignBranch_t *SelectReadyTapeBranch(void)
{
    const TapeAlignBranch_t *branches;
    uint8_t count;
    uint8_t i;

    if (alignAxis == MOVEMENT_AXIS_VERTICAL) {
        branches = VerticalBranches;
        count = (uint8_t) (sizeof(VerticalBranches) / sizeof(VerticalBranches[0]));
    } else {
        branches = HorizontalBranches;
        count = (uint8_t) (sizeof(HorizontalBranches) / sizeof(HorizontalBranches[0]));
    }

    for (i = 0u; i < count; i++) {
        uint8_t pairMask = TapeOffMaskForBranch(&branches[i]);
        if ((tapeOffFlags & pairMask) == pairMask) {
            return &branches[i];
        }
    }

    return (const TapeAlignBranch_t *) 0;
}

static uint8_t StartTapeBranch(const TapeAlignBranch_t *branch,
        AlignState_t *nextState)
{
    float headingError;

    activeTapeBranch = *branch;
    straightTranslateDir = STRAIGHT_TRANSLATE_NONE;
    *nextState = TapeTurnStateForHeading();
#if ALIGN_LOG_ENABLED
    /* New branch: clear the change tracker so the first turn direction below
     * always prints, then announce the selected branch + target turn state. */
    lastAlignAction = "none";
    headingError = RobotIMU_GetHeadingErrorToZeroDeg();
    printf("[ALIGN] branch sensors=%u+%u off pivot=tape%u headingError=",
            (unsigned int) branch->sensorA,
            (unsigned int) branch->sensorB,
            (unsigned int) branch->pivotSensor);
    PrintFixedDeg(headingError);
    printf(" -> %s\r\n", StateNames[*nextState]);
#else
    (void) headingError;
#endif
    return TRUE;
}

static AlignState_t TapeTurnStateForHeading(void)
{
    float error = RobotIMU_GetHeadingErrorToZeroDeg();

    if (error > TAPE_ALIGN_HEADING_STRAIGHT_DEG) {
        return TapeTurnLeftState;
    }
    if (error < -TAPE_ALIGN_HEADING_STRAIGHT_DEG) {
        return TapeTurnRightState;
    }
    return TapeStraightPivotState;
}

static void RefreshTapeOffFlagsFromHardware(void)
{
    uint8_t sensor;

    tapeOffFlags = 0u;
    for (sensor = 1u; sensor <= 5u; sensor++) {
        if ((RobotPlugPlay_IsTapeEnabled(sensor) == TRUE) &&
                (RobotSensors_IsTapeOn((TapeSensor_t) sensor) == FALSE)) {
            tapeOffFlags |= TapeMaskForSensor(sensor);
        }
    }
}

static uint8_t UpdateTapeFlagsFromChangedMask(uint8_t currentMask,
        uint8_t changedMask)
{
    uint8_t sensor;
    uint8_t mask;
    uint8_t updated = FALSE;

    changedMask &= TAPE_SENSOR_ALL_MASK;
    currentMask &= TAPE_SENSOR_ALL_MASK;
    for (sensor = 1u; sensor <= 5u; sensor++) {
        mask = TapeMaskForSensor(sensor);
        if ((changedMask & mask) == 0u) {
            continue;
        }
        if ((currentMask & mask) == 0u) {
            tapeOffFlags |= mask;
        } else {
            tapeOffFlags &= (uint8_t) ~mask;
        }
        updated = TRUE;
    }
    return updated;
}

static uint8_t CompleteIfRealignedTapeOn(ES_Event *event)
{
    uint8_t currentMask;
    uint8_t changedMask;

    if (event->EventType != TapeChangedEvent) {
        return FALSE;
    }

    currentMask = TAPE_EVENT_CURRENT_MASK(event->EventParam);
    changedMask = TAPE_EVENT_CHANGED_MASK(event->EventParam);
    UpdateTapeFlagsFromChangedMask(currentMask, changedMask);
    if ((currentMask & changedMask & activeTapeBranch.realignMask) == 0u) {
        event->EventType = ES_NO_EVENT;
        return FALSE;
    }
    RobotMotion_Stop();
    event->EventType = RealignedEvent;
    event->EventParam = ALIGN_REALIGNED_SOURCE_SENSOR;
    return TRUE;
}

static TurnPivot_t PivotForTapeSensor(uint8_t sensorNumber)
{
    switch (sensorNumber) {
    case 1u:
        return TURN_PIVOT_FRONT_CENTER;
    case 2u:
        return TURN_PIVOT_BACK_CENTER;
    case 3u:
        return TURN_PIVOT_LEFT_CENTER;
    case 4u:
        return TURN_PIVOT_RIGHT_CENTER;
    case 5u:
    default:
        return TURN_PIVOT_CENTER;
    }
}

/* Re-evaluate the heading-straight pivot from live tape readings. While pivoting
 * left about center, the first edge sensor to find the line picks the translate
 * direction that will carry center sensor 5 onto the line:
 *   horizontal: tape3 -> reverse, tape4 -> forward
 *   vertical:   tape1 -> strafe-left, tape2 -> strafe-right
 * If center sensor 5 is already on the line there is nothing to translate. */
static StraightDecision_t EvaluateStraightPivot(void)
{
    if (RobotSensors_IsTapeOn(TAPE_SENSOR_5) == TRUE) {
        return STRAIGHT_DONE;
    }

    if (alignAxis == MOVEMENT_AXIS_HORIZONTAL) {
        if (RobotSensors_IsTapeOn(TAPE_SENSOR_3) == TRUE) {
            straightTranslateDir = STRAIGHT_TRANSLATE_REVERSE;
            return STRAIGHT_GO_TRANSLATE;
        }
        if (RobotSensors_IsTapeOn(TAPE_SENSOR_4) == TRUE) {
            straightTranslateDir = STRAIGHT_TRANSLATE_FORWARD;
            return STRAIGHT_GO_TRANSLATE;
        }
    } else {
        if (RobotSensors_IsTapeOn(TAPE_SENSOR_1) == TRUE) {
            straightTranslateDir = STRAIGHT_TRANSLATE_STRAFE_LEFT;
            return STRAIGHT_GO_TRANSLATE;
        }
        if (RobotSensors_IsTapeOn(TAPE_SENSOR_2) == TRUE) {
            straightTranslateDir = STRAIGHT_TRANSLATE_STRAFE_RIGHT;
            return STRAIGHT_GO_TRANSLATE;
        }
    }

    return STRAIGHT_KEEP_PIVOT;
}

static void DriveStraightTranslate(void)
{
    switch (straightTranslateDir) {
    case STRAIGHT_TRANSLATE_FORWARD:
        RobotMotion_Forward(MOTOR_SPEED_IPS);
        LogAlignAction("straight-forward", RobotIMU_GetHeadingErrorToZeroDeg());
        break;
    case STRAIGHT_TRANSLATE_REVERSE:
        RobotMotion_Reverse(MOTOR_SPEED_IPS);
        LogAlignAction("straight-reverse", RobotIMU_GetHeadingErrorToZeroDeg());
        break;
    case STRAIGHT_TRANSLATE_STRAFE_LEFT:
        RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
        LogAlignAction("straight-strafe-left", RobotIMU_GetHeadingErrorToZeroDeg());
        break;
    case STRAIGHT_TRANSLATE_STRAFE_RIGHT:
        RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
        LogAlignAction("straight-strafe-right", RobotIMU_GetHeadingErrorToZeroDeg());
        break;
    case STRAIGHT_TRANSLATE_NONE:
    default:
        RobotMotion_Stop();
        break;
    }
}

static void DriveTapeTurn(uint8_t turnLeft)
{
    TurnPivot_t pivot = PivotForTapeSensor(activeTapeBranch.pivotSensor);

    if (turnLeft == TRUE) {
        RobotMotion_TurnLeftAbout(pivot, TURN_SPEED_IPS);
        LogAlignAction("tape-turn-left", RobotIMU_GetHeadingErrorToZeroDeg());
    } else {
        RobotMotion_TurnRightAbout(pivot, TURN_SPEED_IPS);
        LogAlignAction("tape-turn-right", RobotIMU_GetHeadingErrorToZeroDeg());
    }
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
