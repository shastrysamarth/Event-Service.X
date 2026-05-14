#include "AlignSubHSM.h"

#include "ES_Framework.h"
#include "RobotDebug.h"
#include "RobotIMU.h"
#include "RobotMotion.h"
#include "RobotPins.h"

typedef enum {
    InitPAlignState,
    PositionAlignState,
    HeadingAlignState,
} AlignState_t;

static const char *StateNames[] = {
    "InitPAlignState",
    "PositionAlignState",
    "HeadingAlignState",
};

static AlignState_t CurrentState = InitPAlignState;
static MovementAxis_t alignAxis = MOVEMENT_AXIS_HORIZONTAL;
static float xRef = 0.0f;
static float yRef = 0.0f;
static uint8_t positionAlignedSamples = 0u;
static uint8_t headingAlignedSamples = 0u;

static float AbsFloat(float value);
static uint8_t StableCheck(uint8_t inThreshold, uint8_t *sampleCounter);

uint8_t InitAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches)
{
    ES_Event returnEvent;

    alignAxis = axis;
    xRef = xRefInches;
    yRef = yRefInches;
    CurrentState = InitPAlignState;
    positionAlignedSamples = 0u;
    headingAlignedSamples = 0u;

    returnEvent = RunAlignSubHSM(INIT_EVENT);
    return (returnEvent.EventType == ES_NO_EVENT) ? TRUE : FALSE;
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
            nextState = PositionAlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case PositionAlignState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            AlignSubHSM_UpdateControl();
            break;
        case PositionRealignedEvent:
            RobotMotion_Stop();
            headingAlignedSamples = 0u;
            nextState = HeadingAlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case RealignedEvent:
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case HeadingAlignState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            AlignSubHSM_UpdateControl();
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
    return (CurrentState == PositionAlignState) ? TRUE : FALSE;
}

uint8_t AlignSubHSM_IsHeadingStage(void)
{
    return (CurrentState == HeadingAlignState) ? TRUE : FALSE;
}

void AlignSubHSM_UpdateControl(void)
{
    float error;

    if (CurrentState == PositionAlignState) {
        if (alignAxis == MOVEMENT_AXIS_HORIZONTAL) {
            error = yRef - RobotIMU_GetYInches();
            if (AbsFloat(error) <= POSITION_THRESHOLD_IN) {
                RobotMotion_Stop();
            } else if (error > 0.0f) {
                RobotMotion_Forward(ALIGN_SPEED_IPS);
            } else {
                RobotMotion_Reverse(ALIGN_SPEED_IPS);
            }
        } else {
            error = xRef - RobotIMU_GetXInches();
            if (AbsFloat(error) <= POSITION_THRESHOLD_IN) {
                RobotMotion_Stop();
            } else if (error > 0.0f) {
                RobotMotion_StrafeRight(ALIGN_SPEED_IPS);
            } else {
                RobotMotion_StrafeLeft(ALIGN_SPEED_IPS);
            }
        }
    } else if (CurrentState == HeadingAlignState) {
        error = RobotIMU_GetHeadingErrorToZeroDeg();
        if (AbsFloat(error) <= HEADING_THRESHOLD_DEG) {
            RobotMotion_Stop();
        } else if (error > 0.0f) {
            RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
        } else {
            RobotMotion_TurnRightAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
        }
    }
}

uint8_t AlignSubHSM_IsPositionAligned(void)
{
    float error;

    if (alignAxis == MOVEMENT_AXIS_HORIZONTAL) {
        error = yRef - RobotIMU_GetYInches();
    } else {
        error = xRef - RobotIMU_GetXInches();
    }

    return StableCheck(AbsFloat(error) <= POSITION_THRESHOLD_IN,
            &positionAlignedSamples);
}

uint8_t AlignSubHSM_IsHeadingAligned(void)
{
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
