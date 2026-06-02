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
    BackUpBehindTapeState,
    ConfirmForwardState,
    RecoverReverseState,
    RecoverTurnState,
} FindFrontTapeState_t;

static const char *StateNames[] = {
    "InitPSubState",
    "WaitForIMUState",
    "SpinLeftState",
    "WaitForBeaconDecreaseState",
    "MoveForwardState",
    "BackUpBehindTapeState",
    "ConfirmForwardState",
    "RecoverReverseState",
    "RecoverTurnState",
};

static FindFrontTapeState_t CurrentState = InitPSubState;
static BoundaryChoice_t boundary_choice = BOUNDARY_TOP;
/* Set once we back up after first hitting tape 5; gates that one-shot maneuver. */
static uint8_t BEHIND_TAPE = FALSE;
/* While confirming a front-edge hit: which sensor confirms square (the OTHER
 * front edge), and which way to pivot if the confirm times out. */
static uint8_t confirmPartnerMask = 0u;
static uint8_t recoverTurnLeft = FALSE;

static uint8_t LiveTapeMask(void);
static uint8_t EventTapeMaskOrLive(ES_Event event);
static uint8_t EventTapeChangedMask(ES_Event event);
static void PostFoundFrontTape(void);

uint8_t InitFindFrontTapeSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = InitPSubState;
    boundary_choice = BOUNDARY_TOP;
    BEHIND_TAPE = FALSE;
    confirmPartnerMask = 0u;
    recoverTurnLeft = FALSE;

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
            BEHIND_TAPE = FALSE;
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
            if (((changedMask & TAPE_SENSOR_5_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_5_MASK) != 0u) &&
                    (BEHIND_TAPE == FALSE)) {
                /* First crossing of the line: back off so the front edge
                 * sensors can re-acquire it cleanly. */
                nextState = BackUpBehindTapeState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (((changedMask & TAPE_SENSOR_3_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_3_MASK) != 0u)) {
                boundary_choice = BOUNDARY_BOTTOM;
                confirmPartnerMask = TAPE_SENSOR_4_MASK;
                recoverTurnLeft = TRUE;
                nextState = ConfirmForwardState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (((changedMask & TAPE_SENSOR_4_MASK) != 0u) &&
                    ((tapeMask & TAPE_SENSOR_4_MASK) != 0u)) {
                boundary_choice = BOUNDARY_TOP;
                confirmPartnerMask = TAPE_SENSOR_3_MASK;
                recoverTurnLeft = FALSE;
                nextState = ConfirmForwardState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case BackUpBehindTapeState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
            ES_Timer_InitTimer(FIND_FRONT_IMU_TIMER, FRONT_TAPE_BACKUP_MS);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == FIND_FRONT_IMU_TIMER) {
                BEHIND_TAPE = TRUE;
                nextState = MoveForwardState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case ConfirmForwardState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Forward(MOTOR_SPEED_IPS);
            /* If the partner edge is already on we are already square. */
            if ((LiveTapeMask() & confirmPartnerMask) != 0u) {
                RobotMotion_Stop();
                PostFoundFrontTape();
            } else {
                ES_Timer_InitTimer(FIND_FRONT_IMU_TIMER, FRONT_TAPE_CONFIRM_MS);
            }
            break;
        case TapeChangedEvent:
            if ((EventTapeMaskOrLive(ThisEvent) & confirmPartnerMask) != 0u) {
                RobotMotion_Stop();
                PostFoundFrontTape();
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == FIND_FRONT_IMU_TIMER) {
                nextState = RecoverReverseState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case RecoverReverseState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Reverse(MOTOR_SPEED_IPS);
            ES_Timer_InitTimer(FIND_FRONT_IMU_TIMER, FRONT_TAPE_BACKUP_MS);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == FIND_FRONT_IMU_TIMER) {
                nextState = RecoverTurnState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case RecoverTurnState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            if (recoverTurnLeft == TRUE) {
                RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
            } else {
                RobotMotion_TurnRightAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
            }
            ES_Timer_InitTimer(FIND_FRONT_IMU_TIMER, FRONT_TAPE_RECOVER_TURN_MS);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == FIND_FRONT_IMU_TIMER) {
                RobotMotion_Stop();
                nextState = MoveForwardState;
                makeTransition = TRUE;
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
