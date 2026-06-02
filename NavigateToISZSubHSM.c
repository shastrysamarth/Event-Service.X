#include "NavigateToISZSubHSM.h"

#include <stdio.h>

#include "AlignSubHSM.h"
#include "ES_Framework.h"
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

typedef enum {
    InitPSubState,
    InitialStrafeLeftState,
    InitialStrafeRightState,
    Reverse1State,
    StrafeRight1State,
    StrafeLeft1State,
    Forward1State,
    Reverse2State,
    Forward2State,
    Forward3State,
    StrafeLeft2State,
    StrafeRight2State,
    Reverse3State,
    AlignState,
} NavigateState_t;

static const char *StateNames[] = {
    "InitPSubState",
    "InitialStrafeLeftState",
    "InitialStrafeRightState",
    "Reverse1State",
    "StrafeRight1State",
    "StrafeLeft1State",
    "Forward1State",
    "Reverse2State",
    "Forward2State",
    "Forward3State",
    "StrafeLeft2State",
    "StrafeRight2State",
    "Reverse3State",
    "AlignState",
};

static NavigateState_t CurrentState = InitPSubState;
static NavigateState_t returnStateAfterAlign = InitPSubState;
static BoundaryChoice_t boundary_choice = BOUNDARY_TOP;
static MovementAxis_t movement_axis = MOVEMENT_AXIS_HORIZONTAL;
static float x_ref = 0.0f;
static float y_ref = 0.0f;
static uint8_t num_tapes_crossed = 0u;
static uint8_t initialStrafeYRefLatched = 0u;

static void SetMovementAxis(MovementAxis_t axis);
static void LatchInitialStrafeYRefOnce(void);
static void StartTapeCounting(void);
static void CountTape5Crossing(void);
static uint8_t IsTop(void);
static uint8_t IsBottom(void);
static uint8_t IsAlignableState(NavigateState_t state);
static AlignMode_t AlignModeForState(NavigateState_t state);
static uint8_t TapeRising(ES_Event event, uint8_t mask);
static uint8_t BumpRising(ES_Event event, uint8_t mask);
static uint8_t BumpFalling(ES_Event event, uint8_t mask);
static uint8_t TapeAlignNeededNow(void);
static uint8_t HandleAlignTriggerEvent(ES_Event *event,
        NavigateState_t *nextState, uint8_t *makeTransition);
static void BeginAlignForState(NavigateState_t state);
static void AcceptCurrentAlignment(uint16_t sourceParam);
static void PostReachedISZ(void);

uint8_t InitNavigateToISZSubHSM(BoundaryChoice_t startingBoundary)
{
    ES_Event returnEvent;

    boundary_choice = startingBoundary;
    movement_axis = MOVEMENT_AXIS_HORIZONTAL;
    x_ref = 0.0f;
    y_ref = 0.0f;
    initialStrafeYRefLatched = 0u;
    num_tapes_crossed = 0u;
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
            RobotIMU_ZeroHeading();
            RobotIMU_ZeroPositionVelocity();
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            nextState = IsTop() ? InitialStrafeLeftState : InitialStrafeRightState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case InitialStrafeLeftState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            LatchInitialStrafeYRefOnce();
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_2_MASK) == TRUE) {
                SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                num_tapes_crossed = 0u;
                nextState = Reverse1State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                /* Keep driving toward the corner. We only command the motor on
                 * ES_ENTRY, so any transient stop (e.g. an align hand-off or an
                 * out-of-order command) would otherwise leave us frozen on the
                 * tape forever. Re-asserting here is a no-op while already
                 * strafing (RobotMotion dedups identical commands). */
                RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            }
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case InitialStrafeRightState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
            LatchInitialStrafeYRefOnce();
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_2_MASK) == TRUE) {
                SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                num_tapes_crossed = 0u;
                nextState = Reverse1State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                /* Re-assert strafe so a transient stop cannot freeze us. */
                RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
            }
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case Reverse1State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
            break;
        case Solenoid4OnEvent:
        case Solenoid5OnEvent:
        case Solenoid6OnEvent:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            StartTapeCounting();
            nextState = IsTop() ? StrafeRight1State : StrafeLeft1State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case BumpChangedEvent:
            if (BumpRising(ThisEvent,
                    BUMP_SENSOR_3_MASK | BUMP_SENSOR_4_MASK) == TRUE) {
                SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                nextState = Forward1State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case TapeChangedEvent:
            if (IsBottom() && (TapeRising(ThisEvent, TAPE_SENSOR_3_MASK) == TRUE)) {
                RobotMotion_Stop();
                RobotIMU_ZeroPositionVelocity();
                RobotMotion_StartDistanceMoveAtSpeed(DISTANCE_AXIS_Y, 1,
                        DISTANCE_FORWARD_TO_ISZ_IN, MOTOR_SPEED_IPS);
                nextState = Forward2State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (IsTop() && (TapeRising(ThisEvent, TAPE_SENSOR_4_MASK) == TRUE)) {
                RobotMotion_Stop();
                RobotIMU_ZeroPositionVelocity();
                RobotMotion_StartDistanceMoveAtSpeed(DISTANCE_AXIS_Y, 1,
                        DISTANCE_FORWARD_TO_ISZ_IN, MOTOR_SPEED_IPS);
                nextState = Forward3State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                /* Re-assert reverse so a transient stop cannot freeze us mid
                 * traverse (we only command the motor on ES_ENTRY otherwise). */
                RobotMotion_Reverse(MOTOR_SPEED_IPS);
            }
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case StrafeRight1State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_5_MASK) == TRUE) {
                CountTape5Crossing();
                if (num_tapes_crossed >= 2u) {
                    boundary_choice = BOUNDARY_BOTTOM;
                    SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                    nextState = Reverse1State;
                    makeTransition = TRUE;
                }
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case BumpChangedEvent:
            if (BumpRising(ThisEvent,
                    BUMP_SENSOR_3_MASK | BUMP_SENSOR_4_MASK) == TRUE) {
                SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                nextState = Forward1State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (BumpRising(ThisEvent,
                    BUMP_SENSOR_1_MASK | BUMP_SENSOR_2_MASK) == TRUE) {
                SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                nextState = Reverse2State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case StrafeLeft1State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_5_MASK) == TRUE) {
                CountTape5Crossing();
                if (num_tapes_crossed >= 2u) {
                    boundary_choice = BOUNDARY_TOP;
                    SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                    nextState = Reverse1State;
                    makeTransition = TRUE;
                }
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case BumpChangedEvent:
            if (BumpRising(ThisEvent,
                    BUMP_SENSOR_3_MASK | BUMP_SENSOR_4_MASK) == TRUE) {
                SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                nextState = Forward1State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (BumpRising(ThisEvent,
                    BUMP_SENSOR_1_MASK | BUMP_SENSOR_2_MASK) == TRUE) {
                SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                nextState = Reverse2State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case Forward1State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            break;
        case BumpChangedEvent:
            if (BumpFalling(ThisEvent,
                    BUMP_SENSOR_3_MASK | BUMP_SENSOR_4_MASK) == TRUE) {
                SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
                StartTapeCounting();
                nextState = IsTop() ? StrafeRight1State : StrafeLeft1State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case Reverse2State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
            break;
        case BumpChangedEvent:
            if (BumpFalling(ThisEvent,
                    BUMP_SENSOR_1_MASK | BUMP_SENSOR_2_MASK) == TRUE) {
                SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
                StartTapeCounting();
                nextState = IsBottom() ? StrafeLeft1State : StrafeRight1State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case Forward2State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            break;
        case DistanceMoveCompleteEvent:
            RobotMotion_Stop();
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            nextState = StrafeLeft2State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case Forward3State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            break;
        case DistanceMoveCompleteEvent:
            RobotMotion_Stop();
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            nextState = StrafeRight2State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case StrafeLeft2State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_1_MASK) == TRUE) {
                RobotMotion_Stop();
                RobotIMU_ZeroPositionVelocity();
                RobotMotion_StartDistanceMoveAtSpeed(DISTANCE_AXIS_Y, -1,
                        DISTANCE_REVERSE_TO_SHOOT_IN, MOTOR_SPEED_IPS);
                nextState = Reverse3State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case StrafeRight2State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
            break;
        case TapeChangedEvent:
            if (TapeRising(ThisEvent, TAPE_SENSOR_1_MASK) == TRUE) {
                RobotMotion_Stop();
                RobotIMU_ZeroPositionVelocity();
                RobotMotion_StartDistanceMoveAtSpeed(DISTANCE_AXIS_Y, -1,
                        DISTANCE_REVERSE_TO_SHOOT_IN, MOTOR_SPEED_IPS);
                nextState = Reverse3State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case Reverse3State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
            break;
        case DistanceMoveCompleteEvent:
            RobotMotion_Stop();
            PostReachedISZ();
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            BeginAlignForState(CurrentState);
            nextState = AlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case AlignState:
        ThisEvent = RunAlignSubHSM(ThisEvent);
        if (ThisEvent.EventType == RealignedEvent) {
            /* Align is done: cancel the shared sweep timer so a stale timeout
             * cannot fire after we hand control back to the strafe/drive state. */
            ES_Timer_StopTimer(NAV_SETTLE_TIMER);
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
    if (IsAlignableState(CurrentState) == FALSE) {
        return FALSE;
    }
    return (AlignModeForState(CurrentState) == ALIGN_MODE_GYRO) ? TRUE : FALSE;
}

uint8_t NavigateToISZ_IsCountingTape5(void)
{
    return ((CurrentState == StrafeRight1State) ||
            (CurrentState == StrafeLeft1State)) ? TRUE : FALSE;
}

MovementAxis_t NavigateToISZ_GetMovementAxis(void)
{
    return movement_axis;
}

float NavigateToISZ_GetXRef(void)
{
    return x_ref;
}

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
    return num_tapes_crossed;
}

static void SetMovementAxis(MovementAxis_t axis)
{
    if (movement_axis != axis) {
        movement_axis = axis;
        if (movement_axis == MOVEMENT_AXIS_HORIZONTAL) {
            y_ref = RobotIMU_GetYInches();
        } else {
            x_ref = RobotIMU_GetXInches();
        }
        RobotIMU_ZeroHeading();
    }
}

static void StartTapeCounting(void)
{
    num_tapes_crossed = 0u;
}

static void CountTape5Crossing(void)
{
    num_tapes_crossed++;
}

static uint8_t IsTop(void)
{
    return (boundary_choice == BOUNDARY_TOP) ? TRUE : FALSE;
}

static uint8_t IsBottom(void)
{
    return (boundary_choice == BOUNDARY_BOTTOM) ? TRUE : FALSE;
}

static uint8_t IsAlignableState(NavigateState_t state)
{
    switch (state) {
    case InitialStrafeLeftState:
    case InitialStrafeRightState:
    case Reverse1State:
    case StrafeRight1State:
    case StrafeLeft1State:
    case Forward1State:
    case Reverse2State:
    case Forward2State:
    case Forward3State:
    case StrafeLeft2State:
    case StrafeRight2State:
    case Reverse3State:
        return TRUE;
    default:
        return FALSE;
    }
}

static AlignMode_t AlignModeForState(NavigateState_t state)
{
    switch (state) {
    case InitialStrafeLeftState:
    case InitialStrafeRightState:
    case Reverse1State:
    case Forward2State:
    case Forward3State:
        return ALIGN_MODE_TAPE;
    default:
        return ALIGN_MODE_GYRO;
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

static uint8_t BumpFalling(ES_Event event, uint8_t mask)
{
    uint8_t currentMask;
    uint8_t changedMask;

    if (event.EventType != BumpChangedEvent) {
        return FALSE;
    }
    currentMask = BUMP_EVENT_CURRENT_MASK(event.EventParam);
    changedMask = BUMP_EVENT_CHANGED_MASK(event.EventParam);
    return ((changedMask & (uint8_t) ~currentMask & mask) != 0u) ? TRUE : FALSE;
}

static uint8_t HandleAlignTriggerEvent(ES_Event *event,
        NavigateState_t *nextState, uint8_t *makeTransition)
{
    AlignMode_t mode;

    if (IsAlignableState(CurrentState) == FALSE) {
        return FALSE;
    }

    mode = AlignModeForState(CurrentState);
    if ((mode == ALIGN_MODE_GYRO) &&
            (event->EventType == MisalignedEvent)) {
        BeginAlignForState(CurrentState);
        *nextState = AlignState;
        *makeTransition = TRUE;
        event->EventType = ES_NO_EVENT;
        return TRUE;
    }

    if (mode == ALIGN_MODE_TAPE) {
        /* Only divert to a tape-align when the robot is genuinely off the line
         * (>=2 of the relevant sensors off). That is exactly the condition the
         * branch selector can act on (it needs a sensor PAIR off). A single
         * sensor blinking off while we strafe across the line must NOT trigger
         * an align that TapeWaitOffState cannot resolve. */
        if (((event->EventType == ES_ENTRY) ||
                (event->EventType == TapeChangedEvent)) &&
                (TapeAlignNeededNow() == TRUE)) {
            BeginAlignForState(CurrentState);
            *nextState = AlignState;
            *makeTransition = TRUE;
            event->EventType = ES_NO_EVENT;
            return TRUE;
        }
        if (event->EventType == MisalignedEvent) {
            event->EventType = ES_NO_EVENT;
            return TRUE;
        }
    }

    return FALSE;
}

static uint8_t TapeAlignNeededNow(void)
{
    uint8_t offCount = 0u;
    uint8_t sensorA;
    uint8_t sensorB;
    uint8_t sensorC = 5u;

    if (movement_axis == MOVEMENT_AXIS_VERTICAL) {
        sensorA = 1u;
        sensorB = 2u;
    } else {
        sensorA = 3u;
        sensorB = 4u;
    }

    if ((RobotPlugPlay_IsTapeEnabled(sensorA) == FALSE) ||
            (RobotPlugPlay_IsTapeEnabled(sensorB) == FALSE) ||
            (RobotPlugPlay_IsTapeEnabled(sensorC) == FALSE)) {
        return FALSE;
    }

    if (RobotSensors_IsTapeOn((TapeSensor_t) sensorA) == FALSE) {
        offCount++;
    }
    if (RobotSensors_IsTapeOn((TapeSensor_t) sensorB) == FALSE) {
        offCount++;
    }
    if (RobotSensors_IsTapeOn((TapeSensor_t) sensorC) == FALSE) {
        offCount++;
    }

    return (offCount >= 2u) ? TRUE : FALSE;
}

static void BeginAlignForState(NavigateState_t state)
{
    AlignMode_t mode = AlignModeForState(state);

    returnStateAfterAlign = state;
    RobotMotion_PauseDistanceMove();
    NAV_TRACE("[NAV] align-start from=%s mode=%s axis=%s "
            "tape s1=%d s2=%d s3=%d s4=%d s5=%d\r\n",
            StateNames[state],
            (mode == ALIGN_MODE_TAPE) ? "tape" : "gyro",
            (movement_axis == MOVEMENT_AXIS_VERTICAL) ? "V" : "H",
            (int) RobotSensors_IsTapeOn(TAPE_SENSOR_1),
            (int) RobotSensors_IsTapeOn(TAPE_SENSOR_2),
            (int) RobotSensors_IsTapeOn(TAPE_SENSOR_3),
            (int) RobotSensors_IsTapeOn(TAPE_SENSOR_4),
            (int) RobotSensors_IsTapeOn(TAPE_SENSOR_5));
    if (mode == ALIGN_MODE_TAPE) {
        InitTapeAlignSubHSM(movement_axis, x_ref, y_ref);
    } else {
        InitGyroAlignSubHSM(movement_axis, x_ref, y_ref);
    }
}

static void AcceptCurrentAlignment(uint16_t sourceParam)
{
    if (sourceParam == (uint16_t) ALIGN_REALIGNED_SOURCE_SENSOR) {
        /* Stable align satisfied in hardware: reset short-horizon odometry and refs together. */
        RobotIMU_ZeroPositionVelocity();
        x_ref = RobotIMU_GetXInches();
        y_ref = RobotIMU_GetYInches();
    }
    RobotMotion_ResumeDistanceMove();
}

static void PostReachedISZ(void)
{
    ES_Event event;

    event.EventType = ReachedISZEvent;
    event.EventParam = 0u;
    PostRobotHSM(event);
}

static void LatchInitialStrafeYRefOnce(void)
{
    if (initialStrafeYRefLatched == 0u) {
        y_ref = RobotIMU_GetYInches();
        initialStrafeYRefLatched = 1u;
    }
}
