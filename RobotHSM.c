#include "RobotHSM.h"

#include "ES_Framework.h"
#include "FindFrontTapeSubHSM.h"
#include "RobotDebug.h"
#include "NavigateToISZSubHSM.h"
#include "RobotHardware.h"
#include "RobotMotion.h"
#include "RobotPins.h"
#include "RobotStepper.h"
#include "ShootingSubHSM.h"
#include "ES_Timers.h"

typedef enum {
    InitPState,
    StartupIgnoreState,
    FindFrontTapeState,
    NavigateToISZState,
    ShootState,
    EndState,
} RobotHSMState_t;

static const char *StateNames[] = {
    "InitPState",
    "StartupIgnoreState",
    "FindFrontTapeState",
    "NavigateToISZState",
    "ShootState",
    "EndState",
};

static RobotHSMState_t CurrentState = InitPState;
static uint8_t MyPriority;

uint8_t InitRobotHSM(uint8_t priority)
{
    MyPriority = priority;
    CurrentState = InitPState;

    return ES_PostToService(MyPriority, INIT_EVENT);
}

uint8_t PostRobotHSM(ES_Event ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event RunRobotHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE;
    RobotHSMState_t nextState = CurrentState;

    ES_Tattle();
    ROBOT_DEBUG_STATE("RobotHSM", StateNames[CurrentState], ThisEvent);

    switch (CurrentState) {
    case InitPState:
        if (ThisEvent.EventType == ES_INIT) {
            nextState = StartupIgnoreState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case StartupIgnoreState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotHardware_StopAllOutputs();
            RobotStepper_Enable();
            ES_Timer_InitTimer(STARTUP_IGNORE_TIMER, STARTUP_IGNORE_MS);
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(STARTUP_IGNORE_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == STARTUP_IGNORE_TIMER) {
                nextState = FindFrontTapeState;
                makeTransition = TRUE;
            }
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        }
        break;

    case FindFrontTapeState:
        if (ThisEvent.EventType == ES_ENTRY) {
            InitFindFrontTapeSubHSM();
        } else {
            ThisEvent = RunFindFrontTapeSubHSM(ThisEvent);
            if (ThisEvent.EventType == FoundFrontTapeEvent) {
                nextState = NavigateToISZState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
        }
        break;

    case NavigateToISZState:
        if (ThisEvent.EventType == ES_ENTRY) {
            InitNavigateToISZSubHSM(FindFrontTape_GetBoundaryChoice());
        } else {
            ThisEvent = RunNavigateToISZSubHSM(ThisEvent);
            if ((ThisEvent.EventType == ReachedISZEvent) ||
                    (ThisEvent.EventType == InsideISZEvent)) {
                nextState = ShootState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
        }
        break;

    case ShootState:
        if (ThisEvent.EventType == ES_ENTRY) {
            InitShootingSubHSM(NavigateToISZ_GetBoundaryChoice());
        } else {
            ThisEvent = RunShootingSubHSM(ThisEvent);
            if (ThisEvent.EventType == DoneEvent) {
                nextState = EndState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
        }
        break;

    case EndState:
        if (ThisEvent.EventType == ES_ENTRY) {
            RobotHardware_StopAllOutputs();
        }
        break;

    default:
        break;
    }

    if (makeTransition == TRUE) {
        RunRobotHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunRobotHSM(ENTRY_EVENT);
        RobotMotion_DebugPrintCurrentCommand("entry");
    }

    ES_Tail();
    return ThisEvent;
}

const char *RobotHSM_GetStateName(void)
{
    return StateNames[CurrentState];
}
