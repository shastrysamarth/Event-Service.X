#include "FindFrontTapeSubHSM.h"

#include "ES_Framework.h"
#include "ES_Timers.h"
#include "RobotDebug.h"
#include "RobotHSM.h"
#include "RobotIMU.h"
#include "RobotMotion.h"
#include "RobotPins.h"
#include "RobotSensors.h"

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
static TurnPivot_t frontTapeTurnPivot = TURN_PIVOT_RIGHT_CENTER;

static uint8_t LiveTapeMask(void);
static uint8_t EventTapeMaskOrLive(ES_Event event);
static uint8_t EventTapeChangedMask(ES_Event event);
static void PostFoundFrontTape(void);

uint8_t InitFindFrontTapeSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = InitPSubState;
    boundary_choice = BOUNDARY_TOP;
    frontTapeTurnPivot = TURN_PIVOT_RIGHT_CENTER;

    returnEvent = RunFindFrontTapeSubHSM(INIT_EVENT);
    return (returnEvent.EventType == ES_NO_EVENT) ? TRUE : FALSE;
}

ES_Event RunFindFrontTapeSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE;
    FindFrontTapeState_t nextState = CurrentState;
    uint8_t tapeMask;
    uint8_t changedMask;

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
            RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, FIND_FRONT_TURN_SPEED_IPS);
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
            RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, FIND_FRONT_TURN_SPEED_IPS);
            break;
        case MaxSignalFoundEvent:
            RobotMotion_Stop();
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
        case TapeChangedEvent:
            tapeMask = EventTapeMaskOrLive(ThisEvent);
            changedMask = EventTapeChangedMask(ThisEvent);
            if (((changedMask & TAPE_SENSOR_4_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_4_MASK) != 0u)) {
                boundary_choice = BOUNDARY_TOP;
                frontTapeTurnPivot = TURN_PIVOT_RIGHT_CENTER;
                nextState = PrepTurnRightState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (((changedMask & TAPE_SENSOR_3_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_3_MASK) != 0u)) {
                boundary_choice = BOUNDARY_BOTTOM;
                frontTapeTurnPivot = TURN_PIVOT_LEFT_CENTER;
                nextState = PrepTurnLeftState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (((changedMask & TAPE_SENSOR_1_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_1_MASK) != 0u)) {
                nextState = SlowMoveForwardState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (((changedMask & TAPE_SENSOR_5_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_5_MASK) != 0u)) {
                nextState = MoveBackwardState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
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
        case TapeChangedEvent:
            tapeMask = EventTapeMaskOrLive(ThisEvent);
            changedMask = EventTapeChangedMask(ThisEvent);
            if (((changedMask & TAPE_SENSOR_4_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_4_MASK) != 0u)) {
                boundary_choice = BOUNDARY_TOP;
                frontTapeTurnPivot = TURN_PIVOT_RIGHT_CENTER;
                nextState = PrepTurnRightState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (((changedMask & TAPE_SENSOR_3_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_3_MASK) != 0u)) {
                boundary_choice = BOUNDARY_BOTTOM;
                frontTapeTurnPivot = TURN_PIVOT_LEFT_CENTER;
                nextState = PrepTurnLeftState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
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
        case TapeChangedEvent:
            tapeMask = EventTapeMaskOrLive(ThisEvent);
            changedMask = EventTapeChangedMask(ThisEvent);
            if (((changedMask & TAPE_SENSOR_4_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_4_MASK) != 0u)) {
                boundary_choice = BOUNDARY_TOP;
                frontTapeTurnPivot = TURN_PIVOT_RIGHT_CENTER;
                nextState = PrepTurnRightState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (((changedMask & TAPE_SENSOR_3_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_3_MASK) != 0u)) {
                boundary_choice = BOUNDARY_BOTTOM;
                frontTapeTurnPivot = TURN_PIVOT_LEFT_CENTER;
                nextState = PrepTurnLeftState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
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
            if ((RobotSensors_IsTapeOn(TAPE_SENSOR_5) == TRUE) && (RobotSensors_IsTapeOn(TAPE_SENSOR_3) == TRUE)) {
                RobotMotion_Stop();
                PostFoundFrontTape();
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                RobotMotion_TurnRightAbout(frontTapeTurnPivot, TURN_SPEED_IPS);
            }
            break;
        case TapeChangedEvent:
            if ((EventTapeMaskOrLive(ThisEvent) & TAPE_SENSOR_5_MASK) != 0u) {
                RobotMotion_Stop();
                PostFoundFrontTape();
                ThisEvent.EventType = ES_NO_EVENT;
            }
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
            if ((RobotSensors_IsTapeOn(TAPE_SENSOR_5) == TRUE) && (RobotSensors_IsTapeOn(TAPE_SENSOR_4) == TRUE)) {
                RobotMotion_Stop();
                PostFoundFrontTape();
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                RobotMotion_TurnLeftAbout(frontTapeTurnPivot, TURN_SPEED_IPS);
            }
            break;
        case TapeChangedEvent:
            if ((EventTapeMaskOrLive(ThisEvent) & TAPE_SENSOR_5_MASK) != 0u) {
                RobotMotion_Stop();
                PostFoundFrontTape();
                ThisEvent.EventType = ES_NO_EVENT;
            }
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
        RobotMotion_DebugPrintCurrentCommand("entry");
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

void FindFrontTape_FastTapeReaction(ES_EventTyp_t eventType, uint8_t tapeMask)
{
    (void) eventType;

    switch (CurrentState) {
    case MoveForwardState:
        if ((tapeMask & TAPE_SENSOR_1_MASK) != 0u) {
            RobotMotion_Forward(MOTOR_SPEED_IPS);
        }
        if ((tapeMask & TAPE_SENSOR_5_MASK) != 0u) {
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
        }
        if ((tapeMask & (TAPE_SENSOR_3_MASK | TAPE_SENSOR_4_MASK)) != 0u) {
            RobotMotion_Stop();
        }
        break;
    case SlowMoveForwardState:
        if ((tapeMask & (TAPE_SENSOR_3_MASK | TAPE_SENSOR_4_MASK)) != 0u) {
            RobotMotion_Stop();
        }
        break;
    case TurnRightState:
        if ((tapeMask & TAPE_SENSOR_5_MASK) != 0u) {
            RobotMotion_Stop();
        }
        break;
    case TurnLeftState:
        if ((tapeMask & TAPE_SENSOR_5_MASK) != 0u) {
            RobotMotion_Stop();
        }
        break;
    default:
        break;
    }
}

const char *FindFrontTape_GetStateName(void)
{
    return StateNames[CurrentState];
}

static uint8_t LiveTapeMask(void)
{
    uint8_t tapeMask = 0u;

    if (RobotSensors_IsTapeOn(TAPE_SENSOR_1) == TRUE) {
        tapeMask |= TAPE_SENSOR_1_MASK;
    }
    if (RobotSensors_IsTapeOn(TAPE_SENSOR_2) == TRUE) {
        tapeMask |= TAPE_SENSOR_2_MASK;
    }
    if (RobotSensors_IsTapeOn(TAPE_SENSOR_3) == TRUE) {
        tapeMask |= TAPE_SENSOR_3_MASK;
    }
    if (RobotSensors_IsTapeOn(TAPE_SENSOR_4) == TRUE) {
        tapeMask |= TAPE_SENSOR_4_MASK;
    }
    if (RobotSensors_IsTapeOn(TAPE_SENSOR_5) == TRUE) {
        tapeMask |= TAPE_SENSOR_5_MASK;
    }

    return tapeMask;
}

static uint8_t EventTapeMaskOrLive(ES_Event event)
{
    if (event.EventType == TapeChangedEvent) {
        return TAPE_EVENT_CURRENT_MASK(event.EventParam);
    }

    return LiveTapeMask();
}

static uint8_t EventTapeChangedMask(ES_Event event)
{
    if (event.EventType == TapeChangedEvent) {
        return TAPE_EVENT_CHANGED_MASK(event.EventParam);
    }
    return 0u;
}

static void PostFoundFrontTape(void)
{
    ES_Event event;

    event.EventType = FoundFrontTapeEvent;
    event.EventParam = (uint16_t) boundary_choice;
    PostRobotHSM(event);
}
