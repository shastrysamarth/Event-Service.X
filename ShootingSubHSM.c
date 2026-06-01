#include "ShootingSubHSM.h"

#include "AlignSubHSM.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "RobotDebug.h"
#include "RobotHSM.h"
#include "RobotLauncher.h"
#include "RobotMotion.h"
#include "RobotPins.h"
#include "RobotStepper.h"

typedef enum {
    InitPShootState,
    AlignBeforeSearchState,
    SearchBeaconMaxState,
    SetLauncherAngleState,
    RunShooterState,
    DoneShootState,
} ShootingState_t;

static const char *StateNames[] = {
    "InitPShootState",
    "AlignBeforeSearchState",
    "SearchBeaconMaxState",
    "SetLauncherAngleState",
    "RunShooterState",
    "DoneShootState",
};

static ShootingState_t CurrentState = InitPShootState;
static uint16_t maxBeaconADC = 0u;
static uint8_t strafeRight = TRUE;

static void DriveBeaconSearchStrafe(void);
static void PostDone(void);

uint8_t InitShootingSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = InitPShootState;
    maxBeaconADC = 0u;
    strafeRight = TRUE;
    RobotStepper_Disable();

    returnEvent = RunShootingSubHSM(INIT_EVENT);
    return (returnEvent.EventType == ES_NO_EVENT) ? TRUE : FALSE;
}

ES_Event RunShootingSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE;
    ShootingState_t nextState = CurrentState;

    ES_Tattle();
    ROBOT_DEBUG_STATE("Shooting", StateNames[CurrentState], ThisEvent);

    switch (CurrentState) {
    case InitPShootState:
        if (ThisEvent.EventType == ES_INIT) {
            RobotStepper_Disable();
            InitGyroAlignSubHSM(MOVEMENT_AXIS_HORIZONTAL, 0.0f, 0.0f);
            nextState = AlignBeforeSearchState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case AlignBeforeSearchState:
        ThisEvent = RunAlignSubHSM(ThisEvent);
        if (ThisEvent.EventType == RealignedEvent) {
            RobotMotion_Stop();
            strafeRight = TRUE;
            nextState = SearchBeaconMaxState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case SearchBeaconMaxState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotStepper_Disable();
            DriveBeaconSearchStrafe();
            break;
        case TapeChangedEvent:
            if (((TAPE_EVENT_CHANGED_MASK(ThisEvent.EventParam) &
                    TAPE_SENSOR_3_MASK) != 0u) &&
                    ((TAPE_EVENT_CURRENT_MASK(ThisEvent.EventParam) &
                    TAPE_SENSOR_3_MASK) != 0u)) {
                strafeRight = TRUE;
                DriveBeaconSearchStrafe();
                ThisEvent.EventType = ES_NO_EVENT;
            }
            if (((TAPE_EVENT_CHANGED_MASK(ThisEvent.EventParam) &
                    TAPE_SENSOR_4_MASK) != 0u) &&
                    ((TAPE_EVENT_CURRENT_MASK(ThisEvent.EventParam) &
                    TAPE_SENSOR_4_MASK) != 0u)) {
                strafeRight = FALSE;
                DriveBeaconSearchStrafe();
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case MisalignedEvent:
            RobotMotion_Stop();
            InitGyroAlignSubHSM(MOVEMENT_AXIS_HORIZONTAL, 0.0f, 0.0f);
            nextState = AlignBeforeSearchState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case MaxSignalFoundEvent:
            maxBeaconADC = ThisEvent.EventParam;
            nextState = SetLauncherAngleState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case SetLauncherAngleState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Stop();
            RobotLauncher_SetAngleFromBeaconADC(maxBeaconADC);
            ThisEvent.EventType = SetEvent;
            PostRobotHSM(ThisEvent);
            break;
        case SetEvent:
            nextState = RunShooterState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case RunShooterState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotLauncher_StartShooter();
            ES_Timer_InitTimer(SHOOT_TIMER, SHOOT_TIME_MS);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == SHOOT_TIMER) {
                RobotLauncher_StopShooter();
                nextState = DoneShootState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case DoneShootState:
        if (ThisEvent.EventType == ES_ENTRY) {
            RobotMotion_Stop();
            RobotLauncher_StopShooter();
            PostDone();
        }
        break;

    default:
        break;
    }

    if (makeTransition == TRUE) {
        RunShootingSubHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunShootingSubHSM(ENTRY_EVENT);
        RobotMotion_DebugPrintCurrentCommand("entry");
    }

    ES_Tail();
    return ThisEvent;
}

uint8_t ShootingSubHSM_IsBeaconSearchActive(void)
{
    return (CurrentState == SearchBeaconMaxState) ? TRUE : FALSE;
}

uint8_t ShootingSubHSM_IsAligning(void)
{
    return (CurrentState == AlignBeforeSearchState) ? TRUE : FALSE;
}

uint8_t ShootingSubHSM_AllowsAlign(void)
{
    return (CurrentState == SearchBeaconMaxState) ? TRUE : FALSE;
}

const char *ShootingSubHSM_GetStateName(void)
{
    return StateNames[CurrentState];
}

uint16_t ShootingSubHSM_GetMaxBeaconADC(void)
{
    return maxBeaconADC;
}

static void DriveBeaconSearchStrafe(void)
{
    if (strafeRight == TRUE) {
        RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
    } else {
        RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
    }
}

static void PostDone(void)
{
    ES_Event event;

    event.EventType = DoneEvent;
    event.EventParam = 0u;
    PostRobotHSM(event);
}
