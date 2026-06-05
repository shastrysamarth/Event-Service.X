#include "RobotHSM.h"

#include "ES_Framework.h"
#include "FindFrontTapeSubHSM.h"
#include "RobotDebug.h"
#include "NavigateToISZSubHSM.h"
#include "RobotHardware.h"
#include "RobotMotion.h"
#include "ShootingSubHSM.h"

typedef enum {
    InitPState,
    FindFrontTapeState,
    NavigateToISZState,
    ShootState,
    EndState,
} RobotHSMState_t;

static const char *StateNames[] = {
    "InitPState",
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
            nextState = FindFrontTapeState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
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
