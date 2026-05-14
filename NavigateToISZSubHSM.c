#include "NavigateToISZSubHSM.h"

#include "AlignSubHSM.h"
#include "ES_Framework.h"
#include "RobotDebug.h"
#include "RobotHSM.h"
#include "RobotIMU.h"
#include "RobotMotion.h"
#include "RobotPins.h"

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
            RobotMotion_StrafeLeft(MOTOR_SPEED_IPS);
            LatchInitialStrafeYRefOnce();
            break;
        case TapeSensor2OnEvent:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            num_tapes_crossed = 0u;
            nextState = Reverse1State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            returnStateAfterAlign = CurrentState;
            InitAlignSubHSM(movement_axis, x_ref, y_ref);
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
            RobotMotion_StrafeRight(MOTOR_SPEED_IPS);
            LatchInitialStrafeYRefOnce();
            break;
        case TapeSensor2OnEvent:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            num_tapes_crossed = 0u;
            nextState = Reverse1State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            returnStateAfterAlign = CurrentState;
            InitAlignSubHSM(movement_axis, x_ref, y_ref);
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
        case BumpSensor3OnEvent:
        case BumpSensor4OnEvent:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            nextState = Forward1State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case TapeSensor3OnEvent:
            if (IsBottom()) {
                RobotMotion_Stop();
                RobotIMU_ZeroPositionVelocity();
                RobotMotion_StartDistanceMove(DISTANCE_AXIS_Y, 1, DISTANCE_FORWARD_TO_ISZ_IN);
                nextState = Forward2State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case TapeSensor4OnEvent:
            if (IsTop()) {
                RobotMotion_Stop();
                RobotIMU_ZeroPositionVelocity();
                RobotMotion_StartDistanceMove(DISTANCE_AXIS_Y, 1, DISTANCE_FORWARD_TO_ISZ_IN);
                nextState = Forward3State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case MisalignedEvent:
            returnStateAfterAlign = CurrentState;
            InitAlignSubHSM(movement_axis, x_ref, y_ref);
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
            RobotMotion_StrafeRight(MOTOR_SPEED_IPS);
            break;
        case TapeSensor5LowToHighEvent:
            CountTape5Crossing();
            if (num_tapes_crossed >= 2u) {
                boundary_choice = BOUNDARY_BOTTOM;
                SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                nextState = Reverse1State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case BumpSensor3OnEvent:
        case BumpSensor4OnEvent:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            nextState = Forward1State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case BumpSensor1OnEvent:
        case BumpSensor2OnEvent:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            nextState = Reverse2State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            returnStateAfterAlign = CurrentState;
            InitAlignSubHSM(movement_axis, x_ref, y_ref);
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
            RobotMotion_StrafeLeft(MOTOR_SPEED_IPS);
            break;
        case TapeSensor5LowToHighEvent:
            CountTape5Crossing();
            if (num_tapes_crossed >= 2u) {
                boundary_choice = BOUNDARY_TOP;
                SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
                nextState = Reverse1State;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case BumpSensor3OnEvent:
        case BumpSensor4OnEvent:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            nextState = Forward1State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case BumpSensor1OnEvent:
        case BumpSensor2OnEvent:
            SetMovementAxis(MOVEMENT_AXIS_VERTICAL);
            nextState = Reverse2State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            returnStateAfterAlign = CurrentState;
            InitAlignSubHSM(movement_axis, x_ref, y_ref);
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
        case BumpSensor3OffEvent:
        case BumpSensor4OffEvent:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            StartTapeCounting();
            nextState = IsTop() ? StrafeRight1State : StrafeLeft1State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            returnStateAfterAlign = CurrentState;
            InitAlignSubHSM(movement_axis, x_ref, y_ref);
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
        case BumpSensor1OffEvent:
        case BumpSensor2OffEvent:
            SetMovementAxis(MOVEMENT_AXIS_HORIZONTAL);
            StartTapeCounting();
            nextState = IsBottom() ? StrafeLeft1State : StrafeRight1State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            returnStateAfterAlign = CurrentState;
            InitAlignSubHSM(movement_axis, x_ref, y_ref);
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
        default:
            break;
        }
        break;

    case StrafeLeft2State:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_StrafeLeft(MOTOR_SPEED_IPS);
            break;
        case TapeSensor1OnEvent:
            RobotMotion_Stop();
            RobotIMU_ZeroPositionVelocity();
            RobotMotion_StartDistanceMove(DISTANCE_AXIS_Y, -1, DISTANCE_REVERSE_TO_SHOOT_IN);
            nextState = Reverse3State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            returnStateAfterAlign = CurrentState;
            InitAlignSubHSM(movement_axis, x_ref, y_ref);
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
            RobotMotion_StrafeRight(MOTOR_SPEED_IPS);
            break;
        case TapeSensor1OnEvent:
            RobotMotion_Stop();
            RobotIMU_ZeroPositionVelocity();
            RobotMotion_StartDistanceMove(DISTANCE_AXIS_Y, -1, DISTANCE_REVERSE_TO_SHOOT_IN);
            nextState = Reverse3State;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MisalignedEvent:
            returnStateAfterAlign = CurrentState;
            InitAlignSubHSM(movement_axis, x_ref, y_ref);
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
        default:
            break;
        }
        break;

    case AlignState:
        ThisEvent = RunAlignSubHSM(ThisEvent);
        if (ThisEvent.EventType == RealignedEvent) {
            AcceptCurrentAlignment(ThisEvent.EventParam);
            nextState = returnStateAfterAlign;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    default:
        break;
    }

    if (makeTransition == TRUE) {
        RunNavigateToISZSubHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunNavigateToISZSubHSM(ENTRY_EVENT);
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
    case StrafeLeft2State:
    case StrafeRight2State:
        return TRUE;
    default:
        return FALSE;
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
    /* Heading offset always refreshed after align (sensor or harness). */
    RobotIMU_ZeroHeading();
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
