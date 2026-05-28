#include "FindFrontTapeSubHSM.h"

#include "ES_Framework.h"
#include "ES_Timers.h"
#include "RobotDebug.h"
#include "RobotHSM.h"
#include "RobotIMU.h"
#include "RobotMotion.h"
#include "RobotPins.h"

typedef enum {
    InitPSubState,
    WaitForIMUState,
    SpinLeftState,
    WaitForBeaconDecreaseState,
    MoveForwardState,
    MoveBackwardState,
    SlowMoveForwardState,
    PrepTurnRightState,
    TurnRightState,
    PrepTurnLeftState,
    TurnLeftState,
} FindFrontTapeState_t;

static const char *StateNames[] = {
    "InitPSubState",
    "WaitForIMUState",
    "SpinLeftState",
    "WaitForBeaconDecreaseState",
    "MoveForwardState",
    "MoveBackwardState",
    "SlowMoveForwardState",
    "PrepTurnRightState",
    "TurnRightState",
    "PrepTurnLeftState",
    "TurnLeftState",
};

static FindFrontTapeState_t CurrentState = InitPSubState;
static BoundaryChoice_t boundary_choice = BOUNDARY_TOP;

static void PostFoundFrontTape(void);

uint8_t InitFindFrontTapeSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = InitPSubState;
    boundary_choice = BOUNDARY_TOP;

    returnEvent = RunFindFrontTapeSubHSM(INIT_EVENT);
    return (returnEvent.EventType == ES_NO_EVENT) ? TRUE : FALSE;
}

ES_Event RunFindFrontTapeSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE;
    FindFrontTapeState_t nextState = CurrentState;

    ES_Tattle();
    ROBOT_DEBUG_STATE("FindFrontTape", StateNames[CurrentState], ThisEvent);

    switch (CurrentState) {
    case InitPSubState:
        if (ThisEvent.EventType == ES_INIT) {
            nextState = WaitForIMUState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case WaitForIMUState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Stop();
            RobotIMU_BeginNDOF();
            RobotIMU_ZeroAll();
            ES_Timer_InitTimer(FIND_FRONT_IMU_TIMER, FIND_FRONT_IMU_SETTLE_MS);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == FIND_FRONT_IMU_TIMER) {
                nextState = SpinLeftState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case SpinLeftState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
            break;
        case BeaconADCIncreaseEvent:
            nextState = WaitForBeaconDecreaseState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case WaitForBeaconDecreaseState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
            break;
        case MaxSignalFoundEvent:
            nextState = MoveForwardState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case MoveForwardState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            break;
        case TapeSensor1OnEvent:
            nextState = SlowMoveForwardState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case TapeSensor5OnEvent:
            nextState = MoveBackwardState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case TapeSensor4OnEvent:
            boundary_choice = BOUNDARY_TOP;
            nextState = PrepTurnRightState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case TapeSensor3OnEvent:
            boundary_choice = BOUNDARY_BOTTOM;
            nextState = PrepTurnLeftState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case MoveBackwardState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
            break;
        case TapeSensor4OnEvent:
            boundary_choice = BOUNDARY_TOP;
            nextState = PrepTurnRightState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case TapeSensor3OnEvent:
            boundary_choice = BOUNDARY_BOTTOM;
            nextState = PrepTurnLeftState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case SlowMoveForwardState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            break;
        case TapeSensor4OnEvent:
            boundary_choice = BOUNDARY_TOP;
            nextState = PrepTurnRightState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case TapeSensor3OnEvent:
            boundary_choice = BOUNDARY_BOTTOM;
            nextState = PrepTurnLeftState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case PrepTurnRightState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Stop();
            ES_Timer_InitTimer(FIND_FRONT_IMU_TIMER, TURN_IMU_SETTLE_MS);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == FIND_FRONT_IMU_TIMER) {
                RobotIMU_ZeroAll();
                nextState = TurnRightState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case TurnRightState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_TurnRightAbout(TURN_PIVOT_RIGHT_CENTER, TURN_SPEED_IPS);
            break;
        case TapeSensor4OffEvent:
            nextState = SlowMoveForwardState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case TapeSensor5OnEvent:
            RobotMotion_Stop();
            PostFoundFrontTape();
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case PrepTurnLeftState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Stop();
            ES_Timer_InitTimer(FIND_FRONT_IMU_TIMER, TURN_IMU_SETTLE_MS);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == FIND_FRONT_IMU_TIMER) {
                RobotIMU_ZeroAll();
                nextState = TurnLeftState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case TurnLeftState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_TurnLeftAbout(TURN_PIVOT_LEFT_CENTER, TURN_SPEED_IPS);
            break;
        case TapeSensor3OffEvent:
            nextState = SlowMoveForwardState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case TapeSensor5OnEvent:
            RobotMotion_Stop();
            PostFoundFrontTape();
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }

    if (makeTransition == TRUE) {
        RunFindFrontTapeSubHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunFindFrontTapeSubHSM(ENTRY_EVENT);
    }

    ES_Tail();
    return ThisEvent;
}

BoundaryChoice_t FindFrontTape_GetBoundaryChoice(void)
{
    return boundary_choice;
}

uint8_t FindFrontTape_IsBeaconSearchActive(void)
{
    return ((CurrentState == SpinLeftState) ||
            (CurrentState == WaitForBeaconDecreaseState)) ? TRUE : FALSE;
}

const char *FindFrontTape_GetStateName(void)
{
    return StateNames[CurrentState];
}

static void PostFoundFrontTape(void)
{
    ES_Event event;

    event.EventType = FoundFrontTapeEvent;
    event.EventParam = (uint16_t) boundary_choice;
    PostRobotHSM(event);
}
