#include "NavigateToISZSubHSM.h"

#include <stdio.h>

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

#if (defined(DEBUG) || defined(ROBOT_DEBUG)) && ROBOT_LOG_NAV
#define NAV_TRACE(...) printf(__VA_ARGS__)
#else
#define NAV_TRACE(...) ((void) 0)
#endif

/*
 * NavigateToISZ (gyro-only rework)
 * --------------------------------
 * On entry the gyro heading reference latched by FindFrontTape is preserved as
 * savedHeadingRefDeg. Every alignment from here on is a GYRO align back to
 * that single saved heading -- the tape-align HSM is no longer used (its code
 * is left in place but deprecated).
 *
 * GyroAlign(X) below means: while in state X, a MisalignedEvent diverts into
 * AlignState (gyro), and a RealignedEvent returns to X. The states that are
 * gyro-aligned are Reverse1, StrafeLeft1, Reverse2, SearchStrafeRight,
 * Reverse3 and SearchStrafeLeft. The fixed 1500ms cross-field strafe states
 * are intentionally not interrupted by align so their timer cannot restart.
 *
 * Flow:
 *   Reverse1 --Tape1 on--> StrafeLeft1 --Tape3 on--> Reverse2
 *
 *   Reverse2 --Tape3 off--> Tape3NudgeRight(500ms) --> Tape3SearchLeft
 *       Tape3SearchLeft --Tape3 on--> Reverse2
 *   Reverse2 --Bump3|4 on--> ForwardBump2(100ms)
 *       ForwardBump2 --timeout--> CrossStrafeRight(1500ms)
 *       CrossStrafeRight --timeout--> SearchStrafeRight
 *       SearchStrafeRight --Tape4 on--> Reverse3
 *   Reverse2 --Tape2 and Tape3 on--> Forward2(200ms) --> StrafeRight5in --> ReachedISZ
 *
 *   Reverse3 --Tape4 off--> Tape4NudgeLeft(500ms) --> Tape4SearchRight
 *       Tape4SearchRight --Tape4 on--> Reverse3
 *   Reverse3 --Bump3|4 on--> ForwardBump3(100ms)
 *       ForwardBump3 --timeout--> CrossStrafeLeft(1500ms)
 *       CrossStrafeLeft --timeout--> SearchStrafeLeft
 *       SearchStrafeLeft --Tape3 on--> Reverse2
 *   Reverse3 --Tape2 and Tape4 on--> Forward3(200ms) --> StrafeLeft5in --> ReachedISZ
 */
typedef enum {
    InitPSubState,
    Reverse1State,
    StrafeLeft1State,
    Reverse2State,
    Tape3NudgeRightState,
    Tape3SearchLeftState,
    ForwardBump2State,
    CrossStrafeRightState,
    SearchStrafeRightState,
    Forward2State,
    StrafeRight5State,
    Reverse3State,
    Tape4NudgeLeftState,
    Tape4SearchRightState,
    ForwardBump3State,
    CrossStrafeLeftState,
    SearchStrafeLeftState,
    Forward3State,
    StrafeLeft5State,
    AlignState,
} NavigateState_t;

static const char *StateNames[] = {
    "InitPSubState",
    "Reverse1State",
    "StrafeLeft1State",
    "Reverse2State",
    "Tape3NudgeRightState",
    "Tape3SearchLeftState",
    "ForwardBump2State",
    "CrossStrafeRightState",
    "SearchStrafeRightState",
    "Forward2State",
    "StrafeRight5State",
    "Reverse3State",
    "Tape4NudgeLeftState",
    "Tape4SearchRightState",
    "ForwardBump3State",
    "CrossStrafeLeftState",
    "SearchStrafeLeftState",
    "Forward3State",
    "StrafeLeft5State",
    "AlignState",
};

static NavigateState_t CurrentState = InitPSubState;
static NavigateState_t returnStateAfterAlign = InitPSubState;
static BoundaryChoice_t boundary_choice = BOUNDARY_TOP;
static MovementAxis_t movement_axis = MOVEMENT_AXIS_VERTICAL;
/* Heading reference saved (and latched into the IMU) on entry. All gyro aligns
 * realign back to this single heading. */
static float savedHeadingRefDeg = 0.0f;
/* DEPRECATED: heading-only align no longer tracks per-axis position refs. */
static float x_ref = 0.0f;
static float y_ref = 0.0f;

static void SetMovementAxis(MovementAxis_t axis);
static uint8_t IsAlignableState(NavigateState_t state);
static uint8_t TapeRising(ES_Event event, uint8_t mask);
static uint8_t TapeFalling(ES_Event event, uint8_t mask);
static uint8_t TapeCurrentHasAll(ES_Event event, uint8_t mask);
static uint8_t BumpRising(ES_Event event, uint8_t mask);
static uint8_t HandleAlignTriggerEvent(ES_Event *event,
        NavigateState_t *nextState, uint8_t *makeTransition);
static TurnPivot_t PivotForState(NavigateState_t state);
static void BeginAlignForState(NavigateState_t state);
static void AcceptCurrentAlignment(uint16_t sourceParam);
static void PostReachedISZ(void);

uint8_t InitNavigateToISZSubHSM(BoundaryChoice_t startingBoundary)
{
    ES_Event returnEvent;

    boundary_choice = startingBoundary;
    movement_axis = MOVEMENT_AXIS_VERTICAL;
    x_ref = 0.0f;
    y_ref = 0.0f;
    savedHeadingRefDeg = 0.0f;
    CurrentState = InitPSubState;

    returnEvent = RunNavigateToISZSubHSM(INIT_EVENT);
    return (returnEvent.EventType == ES_NO_EVENT) ? TRUE : FALSE;
}

ES_Event RunNavigateToISZSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE;
    NavigateState_t nextState = CurrentState;

    ES_Tattle();
    ROBOT_DEBUG_STATE("NavigateToISZ", StateNames[CurrentState], ThisEvent);

#if defined(DEBUG) || defined(ROBOT_DEBUG)
    if ((ThisEvent.EventType != ES_ENTRY) && (ThisEvent.EventType != ES_EXIT) &&
            (ThisEvent.EventType != ES_INIT) &&
            (ThisEvent.EventType != ES_NO_EVENT)) {
        NAV_TRACE("[NAV] evt=%s param=0x%X state=%s\r\n",
                EventNames[ThisEvent.EventType],
                (unsigned int) ThisEvent.EventParam,
                StateNames[CurrentState]);
    }
#endif

    if (HandleAlignTriggerEvent(&ThisEvent, &nextState, &makeTransition) == FALSE) {
    switch (CurrentState) {
    case InitPSubState:
        if (ThisEvent.EventType == ES_INIT) {
            RobotIMU_EnsureNDOF();
            RobotIMU_Update();
            savedHeadingRefDeg = RobotIMU_GetReferenceHeadingDeg();
            NAV_TRACE("[NAV] entry: preserving savedHeadingRef\r\n");
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            nextState = Reverse1State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case Reverse1State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_1_MASK) == TRUE) {
                nextState = StrafeLeft1State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case StrafeLeft1State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_3_MASK) == TRUE) {
                nextState = Reverse2State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case Reverse2State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeCurrentHasAll(ThisEvent,
                    TAPE_SENSOR_2_MASK | TAPE_SENSOR_3_MASK) == TRUE) {
                nextState = Forward2State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (TapeFalling(ThisEvent, TAPE_SENSOR_3_MASK) == TRUE) {
                nextState = Tape3NudgeRightState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case BumpChangedEvent:
            if (BumpRising(ThisEvent,
                    BUMP_SENSOR_3_MASK | BUMP_SENSOR_4_MASK) == TRUE) {
                nextState = ForwardBump2State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case Tape3NudgeRightState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
            ES_Timer_InitTimer(NAV_SETTLE_TIMER, NAV_TAPE_RECOVERY_NUDGE_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_3_MASK) == TRUE) {
                nextState = Reverse2State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = Tape3SearchLeftState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case Tape3SearchLeftState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_3_MASK) == TRUE) {
                nextState = Reverse2State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case ForwardBump2State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            ES_Timer_InitTimer(NAV_SETTLE_TIMER, NAV_FORWARD_AFTER_BUMP_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = CrossStrafeRightState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case CrossStrafeRightState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
            ES_Timer_InitTimer(NAV_SETTLE_TIMER, NAV_BUMP_CROSS_STRAFE_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = SearchStrafeRightState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case SearchStrafeRightState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_4_MASK) == TRUE) {
                nextState = Reverse3State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case Forward2State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            ES_Timer_InitTimer(NAV_SETTLE_TIMER, NAV_FORWARD_TO_ISZ_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = StrafeRight5State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case StrafeRight5State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StartDistanceMoveAtSpeed(DISTANCE_AXIS_X, 1,
                    NAV_FINAL_STRAFE_IN, STRAFE_SPEED_IPS);
            RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
            break;
        case DistanceMoveCompleteEvent:
            RobotMotion_Stop();
            PostReachedISZ();
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case Reverse3State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeCurrentHasAll(ThisEvent,
                    TAPE_SENSOR_2_MASK | TAPE_SENSOR_4_MASK) == TRUE) {
                nextState = Forward3State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (TapeFalling(ThisEvent, TAPE_SENSOR_4_MASK) == TRUE) {
                nextState = Tape4NudgeLeftState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case BumpChangedEvent:
            if (BumpRising(ThisEvent,
                    BUMP_SENSOR_3_MASK | BUMP_SENSOR_4_MASK) == TRUE) {
                nextState = ForwardBump3State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case Tape4NudgeLeftState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            ES_Timer_InitTimer(NAV_SETTLE_TIMER, NAV_TAPE_RECOVERY_NUDGE_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_4_MASK) == TRUE) {
                nextState = Reverse3State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = Tape4SearchRightState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case Tape4SearchRightState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_4_MASK) == TRUE) {
                nextState = Reverse3State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case ForwardBump3State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            ES_Timer_InitTimer(NAV_SETTLE_TIMER, NAV_FORWARD_AFTER_BUMP_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = CrossStrafeLeftState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case CrossStrafeLeftState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            ES_Timer_InitTimer(NAV_SETTLE_TIMER, NAV_BUMP_CROSS_STRAFE_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = SearchStrafeLeftState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case SearchStrafeLeftState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_3_MASK) == TRUE) {
                nextState = Reverse2State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case Forward3State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            ES_Timer_InitTimer(NAV_SETTLE_TIMER, NAV_FORWARD_TO_ISZ_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = StrafeLeft5State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case StrafeLeft5State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            RobotMotion_StartDistanceMoveAtSpeed(DISTANCE_AXIS_X, -1,
                    NAV_FINAL_STRAFE_IN, STRAFE_SPEED_IPS);
            RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            break;
        case DistanceMoveCompleteEvent:
            RobotMotion_Stop();
            PostReachedISZ();
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case AlignState:
        ThisEvent = RunAlignSubHSM(ThisEvent);
        if (ThisEvent.EventType == RealignedEvent) {
            AcceptCurrentAlignment(ThisEvent.EventParam);
            NAV_TRACE("[NAV] align-done -> %s (src=%u)\r\n",
                    StateNames[returnStateAfterAlign],
                    (unsigned int) ThisEvent.EventParam);
            nextState = returnStateAfterAlign;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    default:
        break;
    }
    }

    if (makeTransition == TRUE) {
        RunNavigateToISZSubHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunNavigateToISZSubHSM(ENTRY_EVENT);
        RobotMotion_DebugPrintCurrentCommand("entry");
    }

    ES_Tail();
    return ThisEvent;
}

uint8_t NavigateToISZ_IsActive(void)
{
    return (CurrentState != InitPSubState) ? TRUE : FALSE;
}

uint8_t NavigateToISZ_IsAligning(void)
{
    return (CurrentState == AlignState) ? TRUE : FALSE;
}

uint8_t NavigateToISZ_AllowsAlign(void)
{
    return IsAlignableState(CurrentState);
}

/* DEPRECATED: the gyro-only rework counts corner tapes (sensor 3/4), not the
 * old sensor-5 sweep, so there is no tape-5 counting stage anymore. */
uint8_t NavigateToISZ_IsCountingTape5(void)
{
    return FALSE;
}

MovementAxis_t NavigateToISZ_GetMovementAxis(void)
{
    return movement_axis;
}

/* DEPRECATED: heading-only align no longer keeps a per-axis position ref. */
float NavigateToISZ_GetXRef(void)
{
    return x_ref;
}

/* DEPRECATED: heading-only align no longer keeps a per-axis position ref. */
float NavigateToISZ_GetYRef(void)
{
    return y_ref;
}

const char *NavigateToISZ_GetStateName(void)
{
    return StateNames[CurrentState];
}

BoundaryChoice_t NavigateToISZ_GetBoundaryChoice(void)
{
    return boundary_choice;
}

uint8_t NavigateToISZ_GetNumTapesCrossed(void)
{
    return 0u;
}

float NavigateToISZ_GetSavedHeadingRefDeg(void)
{
    return savedHeadingRefDeg;
}

/* Display/debug only: alignment now keys purely off the saved heading, so the
 * movement axis no longer re-zeros heading the way the old rework did. */
static void SetMovementAxis(MovementAxis_t axis)
{
    movement_axis = axis;
}

static uint8_t IsAlignableState(NavigateState_t state)
{
    switch (state) {
    case Reverse1State:
    case StrafeLeft1State:
    case Reverse2State:
    case SearchStrafeRightState:
    case Reverse3State:
    case SearchStrafeLeftState:
        return TRUE;
    default:
        return FALSE;
    }
}

static uint8_t TapeRising(ES_Event event, uint8_t mask)
{
    uint8_t currentMask;
    uint8_t changedMask;

    if (event.EventType != TapeChangedEvent) {
        return FALSE;
    }
    currentMask = TAPE_EVENT_CURRENT_MASK(event.EventParam);
    changedMask = TAPE_EVENT_CHANGED_MASK(event.EventParam);
    return ((currentMask & changedMask & mask) != 0u) ? TRUE : FALSE;
}

static uint8_t TapeFalling(ES_Event event, uint8_t mask)
{
    uint8_t currentMask;
    uint8_t changedMask;

    if (event.EventType != TapeChangedEvent) {
        return FALSE;
    }
    currentMask = TAPE_EVENT_CURRENT_MASK(event.EventParam);
    changedMask = TAPE_EVENT_CHANGED_MASK(event.EventParam);
    return ((changedMask & (uint8_t) ~currentMask & mask) != 0u) ? TRUE : FALSE;
}

static uint8_t TapeCurrentHasAll(ES_Event event, uint8_t mask)
{
    uint8_t currentMask;

    if (event.EventType != TapeChangedEvent) {
        return FALSE;
    }
    currentMask = TAPE_EVENT_CURRENT_MASK(event.EventParam);
    return ((currentMask & mask) == mask) ? TRUE : FALSE;
}

static uint8_t BumpRising(ES_Event event, uint8_t mask)
{
    uint8_t currentMask;
    uint8_t changedMask;

    if (event.EventType != BumpChangedEvent) {
        return FALSE;
    }
    currentMask = BUMP_EVENT_CURRENT_MASK(event.EventParam);
    changedMask = BUMP_EVENT_CHANGED_MASK(event.EventParam);
    return ((currentMask & changedMask & mask) != 0u) ? TRUE : FALSE;
}

static uint8_t HandleAlignTriggerEvent(ES_Event *event,
        NavigateState_t *nextState, uint8_t *makeTransition)
{
    if (IsAlignableState(CurrentState) == FALSE) {
        return FALSE;
    }

    if (event->EventType == MisalignedEvent) {
        BeginAlignForState(CurrentState);
        *nextState = AlignState;
        *makeTransition = TRUE;
        event->EventType = ES_NO_EVENT;
        return TRUE;
    }

    return FALSE;
}

/* Pivot the gyro correction about the tape sensor the state is tracking:
 * tape 3 (left of center) -> left-center, tape 4 (right of center) ->
 * right-center, and an ambiguous state (e.g. the first reverse) -> the center
 * tape (sensor 5). */
static TurnPivot_t PivotForState(NavigateState_t state)
{
    switch (state) {
    case StrafeLeft1State:
    case Reverse2State:
    case SearchStrafeLeftState:
        return TURN_PIVOT_LEFT_CENTER;
    case Reverse3State:
    case SearchStrafeRightState:
        return TURN_PIVOT_RIGHT_CENTER;
    case Reverse1State:
    default:
        return TURN_PIVOT_CENTER;
    }
}

static void BeginAlignForState(NavigateState_t state)
{
    returnStateAfterAlign = state;
    RobotMotion_PauseDistanceMove();
    /* Active-brake the current drive command before the align takes over. */
    RobotMotion_Stop();
    NAV_TRACE("[NAV] gyro-align-start from=%s\r\n", StateNames[state]);
    /* Gyro align only uses the latched reference heading; the x/y refs are
     * passed for API compatibility and ignored by the heading control. */
    InitGyroAlignSubHSM(movement_axis, PivotForState(state), x_ref, y_ref);
}

static void AcceptCurrentAlignment(uint16_t sourceParam)
{
    /* Heading is back on the saved reference; nothing to re-anchor. Resume any
     * paused distance move (e.g. the final ISZ strafe). */
    (void) sourceParam;
    RobotMotion_ResumeDistanceMove();
}

static void PostReachedISZ(void)
{
    ES_Event event;

    event.EventType = ReachedISZEvent;
    event.EventParam = 0u;
    PostRobotHSM(event);
}
