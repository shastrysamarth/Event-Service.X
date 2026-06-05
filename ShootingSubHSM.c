#include "ShootingSubHSM.h"

#include "AlignSubHSM.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "RobotDebug.h"
#include "RobotHSM.h"
#include "RobotLauncher.h"
#include "RobotMotion.h"
#include "RobotPins.h"
#include "RobotSensors.h"
#include "RobotStepper.h"

typedef enum {
    InitPShootState,
    AlignBeforeSearchState,
    SearchBeaconMaxState,
    SearchTimedStrafeState,
    SearchUntilTapeState,
    Tape5ReverseEscapeState,
    InitialStepperFallState,
    SetLauncherAngleState,
    RunShooterState,
    ReloadRaiseState,
    ReloadHoldState,
    ReloadFallState,
    DoneShootState,
} ShootingState_t;

static const char *StateNames[] = {
    "InitPShootState",
    "AlignBeforeSearchState",
    "SearchBeaconMaxState",
    "SearchTimedStrafeState",
    "SearchUntilTapeState",
    "Tape5ReverseEscapeState",
    "InitialStepperFallState",
    "SetLauncherAngleState",
    "RunShooterState",
    "ReloadRaiseState",
    "ReloadHoldState",
    "ReloadFallState",
    "DoneShootState",
};

static ShootingState_t CurrentState = InitPShootState;
static uint16_t maxBeaconADC = 0u;
static uint8_t strafeRight = TRUE;
static uint8_t searchTargetTapeMask = TAPE_SENSOR_4_MASK;
static BoundaryChoice_t boundary_choice = BOUNDARY_BOTTOM;
static ShootingState_t searchResumeState = SearchTimedStrafeState;
static ShootingState_t returnStateAfterAlign = SearchBeaconMaxState;
static uint16_t searchTimedRemainingMs = SHOOT_BEACON_SEARCH_STRAFE_MS;
static uint32_t searchTimedStartMs = 0u;
static uint32_t shootCycleStartMs = 0u;
static int16_t savedLauncherStep = LAUNCHER_STEPPER_HOME_STEP;

static uint8_t LiveTapeMask(void);
static uint8_t TapeEventCurrentMask(ES_Event event);
static uint8_t TapeEventChangedMask(ES_Event event);
static void ChooseInitialSearchDirection(void);
static void SetSearchDirectionForBoundary(BoundaryChoice_t boundary);
static void SetSearchDirectionFromReachedTarget(void);
static void SetSearchDirection(uint8_t goRight);
static void DriveBeaconSearchStrafe(void);
static uint8_t Tape5TurnedOn(ES_Event event);
static uint8_t SearchTargetTapeOn(ES_Event event);
static uint8_t HandleBeaconSearchEvent(ES_Event event,
        ShootingState_t *nextState, uint8_t *makeTransition);
static void PauseForTape5Escape(ShootingState_t resumeState);
static void ResumeAfterTape5Escape(void);
static void PauseForGyroAlign(void);
static void StartGyroAlign(void);
static uint32_t ShootCycleRemainingMs(void);
static uint8_t ShootCycleExpired(void);
static void PostDone(void);

uint8_t InitShootingSubHSM(BoundaryChoice_t startingBoundary)
{
    ES_Event returnEvent;

    CurrentState = InitPShootState;
    maxBeaconADC = 0u;
    boundary_choice = startingBoundary;
    SetSearchDirectionForBoundary(boundary_choice);
    searchResumeState = SearchTimedStrafeState;
    returnStateAfterAlign = SearchBeaconMaxState;
    searchTimedRemainingMs = SHOOT_BEACON_SEARCH_STRAFE_MS;
    searchTimedStartMs = 0u;
    shootCycleStartMs = 0u;
    savedLauncherStep = LAUNCHER_STEPPER_HOME_STEP;
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
            SetSearchDirectionForBoundary(boundary_choice);
            returnStateAfterAlign = SearchBeaconMaxState;
            shootCycleStartMs = 0u;
            savedLauncherStep = LAUNCHER_STEPPER_HOME_STEP;
            StartGyroAlign();
            nextState = AlignBeforeSearchState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case AlignBeforeSearchState:
        ThisEvent = RunAlignSubHSM(ThisEvent);
        if (ThisEvent.EventType == RealignedEvent) {
            RobotMotion_Stop();
            nextState = returnStateAfterAlign;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case SearchBeaconMaxState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotStepper_Disable();
            if (RobotSensors_IsTapeOn(TAPE_SENSOR_5) == TRUE) {
                searchResumeState = SearchBeaconMaxState;
                nextState = Tape5ReverseEscapeState;
            } else {
                ChooseInitialSearchDirection();
                searchTimedRemainingMs = SHOOT_BEACON_SEARCH_STRAFE_MS;
                nextState = SearchTimedStrafeState;
            }
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case SearchTimedStrafeState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotStepper_Disable();
            DriveBeaconSearchStrafe();
            searchTimedStartMs = ES_Timer_GetTime();
            if (ES_Timer_StartTimer(SHOOT_TIMER) == -1) {
                ES_Timer_InitTimer(SHOOT_TIMER, searchTimedRemainingMs);
            }
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(SHOOT_TIMER);
            break;
        case TapeChangedEvent:
            if (Tape5TurnedOn(ThisEvent) == TRUE) {
                PauseForTape5Escape(SearchTimedStrafeState);
                nextState = Tape5ReverseEscapeState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == SHOOT_TIMER) {
                ES_Timer_SetTimer(SHOOT_TIMER, searchTimedRemainingMs);
                searchTimedRemainingMs = SHOOT_BEACON_SEARCH_STRAFE_MS;
                nextState = SearchUntilTapeState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            if (HandleBeaconSearchEvent(ThisEvent, &nextState,
                    &makeTransition) == TRUE) {
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        }
        break;

    case SearchUntilTapeState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotStepper_Disable();
            if ((LiveTapeMask() & searchTargetTapeMask) != 0u) {
                SetSearchDirectionFromReachedTarget();
                searchTimedRemainingMs = SHOOT_BEACON_SEARCH_STRAFE_MS;
                nextState = SearchTimedStrafeState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                DriveBeaconSearchStrafe();
            }
            break;
        case TapeChangedEvent:
            if (Tape5TurnedOn(ThisEvent) == TRUE) {
                PauseForTape5Escape(SearchUntilTapeState);
                nextState = Tape5ReverseEscapeState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else if (SearchTargetTapeOn(ThisEvent) == TRUE) {
                SetSearchDirectionFromReachedTarget();
                searchTimedRemainingMs = SHOOT_BEACON_SEARCH_STRAFE_MS;
                nextState = SearchTimedStrafeState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            if (HandleBeaconSearchEvent(ThisEvent, &nextState,
                    &makeTransition) == TRUE) {
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        }
        break;

    case Tape5ReverseEscapeState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            if (RobotSensors_IsTapeOn(TAPE_SENSOR_5) == FALSE) {
                ResumeAfterTape5Escape();
                nextState = searchResumeState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                RobotMotion_Reverse(MOTOR_SPEED_IPS);
            }
            break;
        case TapeChangedEvent:
            if (((TapeEventChangedMask(ThisEvent) &
                    TAPE_SENSOR_5_MASK) != 0u) &&
                    ((TapeEventCurrentMask(ThisEvent) &
                    TAPE_SENSOR_5_MASK) == 0u)) {
                ResumeAfterTape5Escape();
                nextState = searchResumeState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case BeaconADCIncreaseEvent:
        case MaxSignalFoundEvent:
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case InitialStepperFallState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Stop();
            RobotLauncher_StopShooter();
            RobotStepper_Disable();
            shootCycleStartMs = ES_Timer_GetTime();
            ES_Timer_InitTimer(SHOOT_TIMER, SHOOT_STEPPER_FALL_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(SHOOT_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == SHOOT_TIMER) {
                RobotStepper_ZeroPosition();
                nextState = SetLauncherAngleState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case SetLauncherAngleState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotMotion_Stop();
            savedLauncherStep = RobotLauncher_GetTargetStepForBeaconADC(maxBeaconADC);
            RobotStepper_MoveToStep(savedLauncherStep);
            ThisEvent.EventType = SetEvent;
            PostRobotHSM(ThisEvent);
            break;
        case SetEvent:
            nextState = (ShootCycleExpired() == TRUE) ?
                    DoneShootState : RunShooterState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case RunShooterState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY: {
            uint32_t remainingMs = ShootCycleRemainingMs();
            if (remainingMs == 0u) {
                nextState = DoneShootState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
                break;
            }
            RobotStepper_Enable();
            RobotLauncher_StartShooter();
            ES_Timer_InitTimer(SHOOT_TIMER,
                    (remainingMs < SHOOTER_RUN_MS) ? remainingMs : SHOOTER_RUN_MS);
            break;
        }
        case ES_EXIT:
            ES_Timer_StopTimer(SHOOT_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == SHOOT_TIMER) {
                RobotLauncher_StopShooter();
                nextState = (ShootCycleExpired() == TRUE) ?
                        DoneShootState : ReloadRaiseState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case ReloadRaiseState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotStepper_MoveToStep(LAUNCHER_STEPPER_RELOAD_STEP);
            nextState = ReloadHoldState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        default:
            break;
        }
        break;

    case ReloadHoldState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotStepper_Enable();
            ES_Timer_InitTimer(SHOOT_TIMER, SHOOT_RELOAD_HOLD_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(SHOOT_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == SHOOT_TIMER) {
                nextState = ReloadFallState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        default:
            break;
        }
        break;

    case ReloadFallState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RobotStepper_Disable();
            ES_Timer_InitTimer(SHOOT_TIMER, SHOOT_STEPPER_FALL_MS);
            break;
        case ES_EXIT:
            ES_Timer_StopTimer(SHOOT_TIMER);
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == SHOOT_TIMER) {
                RobotStepper_ZeroPosition();
                nextState = (ShootCycleExpired() == TRUE) ?
                        DoneShootState : SetLauncherAngleState;
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
            RobotStepper_Disable();
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
    return ((CurrentState == SearchBeaconMaxState) ||
            (CurrentState == SearchTimedStrafeState) ||
            (CurrentState == SearchUntilTapeState)) ? TRUE : FALSE;
}

uint8_t ShootingSubHSM_IsAligning(void)
{
    return (CurrentState == AlignBeforeSearchState) ? TRUE : FALSE;
}

uint8_t ShootingSubHSM_AllowsAlign(void)
{
    return ((CurrentState == SearchTimedStrafeState) ||
            (CurrentState == SearchUntilTapeState)) ? TRUE : FALSE;
}

const char *ShootingSubHSM_GetStateName(void)
{
    return StateNames[CurrentState];
}

uint16_t ShootingSubHSM_GetMaxBeaconADC(void)
{
    return maxBeaconADC;
}

static uint8_t LiveTapeMask(void)
{
    uint8_t mask = 0u;

    if (RobotSensors_IsTapeOn(TAPE_SENSOR_1) == TRUE) {
        mask |= TAPE_SENSOR_1_MASK;
    }
    if (RobotSensors_IsTapeOn(TAPE_SENSOR_2) == TRUE) {
        mask |= TAPE_SENSOR_2_MASK;
    }
    if (RobotSensors_IsTapeOn(TAPE_SENSOR_3) == TRUE) {
        mask |= TAPE_SENSOR_3_MASK;
    }
    if (RobotSensors_IsTapeOn(TAPE_SENSOR_4) == TRUE) {
        mask |= TAPE_SENSOR_4_MASK;
    }
    if (RobotSensors_IsTapeOn(TAPE_SENSOR_5) == TRUE) {
        mask |= TAPE_SENSOR_5_MASK;
    }

    return mask;
}

static uint8_t TapeEventCurrentMask(ES_Event event)
{
    return TAPE_EVENT_CURRENT_MASK(event.EventParam);
}

static uint8_t TapeEventChangedMask(ES_Event event)
{
    return TAPE_EVENT_CHANGED_MASK(event.EventParam);
}

static void ChooseInitialSearchDirection(void)
{
    uint8_t tapeMask = LiveTapeMask();

    if ((tapeMask & TAPE_SENSOR_3_MASK) != 0u) {
        boundary_choice = BOUNDARY_BOTTOM;
        SetSearchDirection(TRUE);
    } else if ((tapeMask & TAPE_SENSOR_4_MASK) != 0u) {
        boundary_choice = BOUNDARY_TOP;
        SetSearchDirection(FALSE);
    } else {
        SetSearchDirectionForBoundary(boundary_choice);
    }
}

static void SetSearchDirectionForBoundary(BoundaryChoice_t boundary)
{
    SetSearchDirection((boundary == BOUNDARY_BOTTOM) ? TRUE : FALSE);
}

static void SetSearchDirectionFromReachedTarget(void)
{
    boundary_choice = (searchTargetTapeMask == TAPE_SENSOR_4_MASK) ?
            BOUNDARY_TOP : BOUNDARY_BOTTOM;
    SetSearchDirectionForBoundary(boundary_choice);
}

static void SetSearchDirection(uint8_t goRight)
{
    strafeRight = goRight ? TRUE : FALSE;
    searchTargetTapeMask = (strafeRight == TRUE) ?
            TAPE_SENSOR_4_MASK : TAPE_SENSOR_3_MASK;
}

static void DriveBeaconSearchStrafe(void)
{
    if (strafeRight == TRUE) {
        RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
    } else {
        RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
    }
}

static uint8_t Tape5TurnedOn(ES_Event event)
{
    if (event.EventType != TapeChangedEvent) {
        return FALSE;
    }
    return (((TapeEventChangedMask(event) & TAPE_SENSOR_5_MASK) != 0u) &&
            ((TapeEventCurrentMask(event) & TAPE_SENSOR_5_MASK) != 0u)) ?
            TRUE : FALSE;
}

static uint8_t SearchTargetTapeOn(ES_Event event)
{
    if (event.EventType != TapeChangedEvent) {
        return FALSE;
    }
    return (((TapeEventChangedMask(event) & searchTargetTapeMask) != 0u) &&
            ((TapeEventCurrentMask(event) & searchTargetTapeMask) != 0u)) ?
            TRUE : FALSE;
}

static uint8_t HandleBeaconSearchEvent(ES_Event event,
        ShootingState_t *nextState, uint8_t *makeTransition)
{
    switch (event.EventType) {
    case BeaconADCIncreaseEvent:
        return TRUE;
    case MaxSignalFoundEvent:
        maxBeaconADC = event.EventParam;
        *nextState = InitialStepperFallState;
        *makeTransition = TRUE;
        return TRUE;
    case MisalignedEvent:
        RobotMotion_Stop();
        PauseForGyroAlign();
        StartGyroAlign();
        returnStateAfterAlign = CurrentState;
        *nextState = AlignBeforeSearchState;
        *makeTransition = TRUE;
        return TRUE;
    default:
        break;
    }
    return FALSE;
}

static void PauseForTape5Escape(ShootingState_t resumeState)
{
    uint32_t now = ES_Timer_GetTime();
    uint32_t elapsed;

    searchResumeState = resumeState;
    if ((resumeState == SearchTimedStrafeState) &&
            (CurrentState == SearchTimedStrafeState)) {
        elapsed = now - searchTimedStartMs;
        searchTimedRemainingMs = (elapsed >= searchTimedRemainingMs) ? 1u :
                (uint16_t) (searchTimedRemainingMs - elapsed);
        ES_Timer_StopTimer(SHOOT_TIMER);
    }
}

static void ResumeAfterTape5Escape(void)
{
    if ((searchResumeState == SearchTimedStrafeState) &&
            (searchTimedRemainingMs == 0u)) {
        searchTimedRemainingMs = 1u;
    }
}

static void PauseForGyroAlign(void)
{
    uint32_t now = ES_Timer_GetTime();
    uint32_t elapsed;

    if (CurrentState == SearchTimedStrafeState) {
        elapsed = now - searchTimedStartMs;
        searchTimedRemainingMs = (elapsed >= searchTimedRemainingMs) ? 1u :
                (uint16_t) (searchTimedRemainingMs - elapsed);
    }
    ES_Timer_StopTimer(SHOOT_TIMER);
}

static void StartGyroAlign(void)
{
    InitGyroAlignSubHSM(MOVEMENT_AXIS_HORIZONTAL, TURN_PIVOT_CENTER, 0.0f, 0.0f);
}

static uint32_t ShootCycleRemainingMs(void)
{
    uint32_t elapsedMs = ES_Timer_GetTime() - shootCycleStartMs;

    return (elapsedMs >= SHOOT_CYCLE_TOTAL_MS) ? 0u :
            (SHOOT_CYCLE_TOTAL_MS - elapsedMs);
}

static uint8_t ShootCycleExpired(void)
{
    return (ShootCycleRemainingMs() == 0u) ? TRUE : FALSE;
}

static void PostDone(void)
{
    ES_Event event;

    event.EventType = DoneEvent;
    event.EventParam = 0u;
    PostRobotHSM(event);
}
