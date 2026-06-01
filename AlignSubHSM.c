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

#include <stdio.h>

typedef enum {
    InitPAlignState,
    GyroHeadingAlignState,
    TapeWaitOffState,
    TapeTurnLeftState,
    TapeTurnRightState,
    TapeStraightRightState,
    TapeStraightLeftState,
} AlignState_t;

typedef struct {
    ES_EventTyp_t comboEvent;
    uint8_t sensorA;
    uint8_t sensorB;
    uint8_t pivotSensor;
    uint8_t realignMask;
} TapeAlignBranch_t;

static const char *StateNames[] = {
    "InitPAlignState",
    "GyroHeadingAlignState",
    "TapeWaitOffState",
    "TapeTurnLeftState",
    "TapeTurnRightState",
    "TapeStraightRightState",
    "TapeStraightLeftState",
};

static const TapeAlignBranch_t VerticalBranches[] = {
    {TapeSensor1And2OffEvent, 1u, 2u, 5u, TAPE_SENSOR_1_MASK | TAPE_SENSOR_2_MASK},
    {TapeSensor1And5OffEvent, 1u, 5u, 2u, TAPE_SENSOR_5_MASK},
    {TapeSensor2And5OffEvent, 2u, 5u, 1u, TAPE_SENSOR_5_MASK},
};

static const TapeAlignBranch_t HorizontalBranches[] = {
    {TapeSensor3And4OffEvent, 3u, 4u, 5u, TAPE_SENSOR_3_MASK | TAPE_SENSOR_4_MASK},
    {TapeSensor3And5OffEvent, 3u, 5u, 4u, TAPE_SENSOR_5_MASK},
    {TapeSensor4And5OffEvent, 4u, 5u, 3u, TAPE_SENSOR_5_MASK},
};

static AlignState_t CurrentState = InitPAlignState;
static AlignMode_t alignMode = ALIGN_MODE_GYRO;
static MovementAxis_t alignAxis = MOVEMENT_AXIS_HORIZONTAL;
static float xRef = 0.0f;
static float yRef = 0.0f;
static uint8_t headingAlignedSamples = 0u;
static uint8_t tapeOffFlags = 0u;
static TapeAlignBranch_t activeTapeBranch = {
    TapeSensor1And2OffEvent, 1u, 2u, 5u,
    TAPE_SENSOR_1_MASK | TAPE_SENSOR_2_MASK
};

static uint8_t InitAlignCommon(AlignMode_t mode, MovementAxis_t axis,
        float xRefInches, float yRefInches);
static float AbsFloat(float value);
static uint8_t StableCheck(uint8_t inThreshold, uint8_t *sampleCounter);
static uint8_t TapeMaskForSensor(uint8_t sensorNumber);
static uint8_t TapeStateMask(void);
static uint8_t TapeOffMaskForBranch(const TapeAlignBranch_t *branch);
static const TapeAlignBranch_t *SelectReadyTapeBranch(void);
static const TapeAlignBranch_t *FindTapeBranchForEvent(ES_EventTyp_t eventType);
static uint8_t StartTapeBranch(const TapeAlignBranch_t *branch,
        AlignState_t *nextState);
static AlignState_t TapeTurnStateForHeading(void);
static void RefreshTapeOffFlagsFromHardware(void);
static uint8_t UpdateTapeFlagsFromEvent(ES_EventTyp_t eventType);
static uint8_t SensorFromTapeEvent(ES_EventTyp_t eventType, uint8_t *sensorNumber);
static uint8_t IsTapeOnEvent(ES_EventTyp_t eventType);
static uint8_t IsTapeOffEvent(ES_EventTyp_t eventType);
static uint8_t IsTapeComboEvent(ES_EventTyp_t eventType);
static uint8_t CompleteIfRealignedTapeOn(ES_Event *event);
static TurnPivot_t PivotForTapeSensor(uint8_t sensorNumber);
static void DriveTapeTurn(uint8_t turnLeft);
static void PostTapeComboEvent(const TapeAlignBranch_t *branch);
static void PrintFixedDeg(float value);

uint8_t InitAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches)
{
    return InitGyroAlignSubHSM(axis, xRefInches, yRefInches);
}

uint8_t InitGyroAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches)
{
    return InitAlignCommon(ALIGN_MODE_GYRO, axis, xRefInches, yRefInches);
}

uint8_t InitTapeAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches)
{
    return InitAlignCommon(ALIGN_MODE_TAPE, axis, xRefInches, yRefInches);
}

ES_Event RunAlignSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE;
    AlignState_t nextState = CurrentState;
    const TapeAlignBranch_t *branch;

    ES_Tattle();
    ROBOT_DEBUG_STATE("Align", StateNames[CurrentState], ThisEvent);

    switch (CurrentState) {
    case InitPAlignState:
        if (ThisEvent.EventType == ES_INIT) {
            nextState = (alignMode == ALIGN_MODE_TAPE) ?
                    TapeWaitOffState : GyroHeadingAlignState;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case GyroHeadingAlignState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            AlignSubHSM_UpdateControl();
            break;
        case PositionRealignedEvent:
            ThisEvent.EventType = ES_NO_EVENT;
            break;
        case RealignedEvent:
            RobotMotion_Stop();
            ThisEvent.EventType = RealignedEvent;
            break;
        default:
            break;
        }
        break;

    case TapeWaitOffState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            RefreshTapeOffFlagsFromHardware();
            branch = SelectReadyTapeBranch();
            if (branch != (const TapeAlignBranch_t *) 0) {
                PostTapeComboEvent(branch);
            }
            break;
        default:
            if (IsTapeComboEvent(ThisEvent.EventType) == TRUE) {
                branch = FindTapeBranchForEvent(ThisEvent.EventType);
                if (branch != (const TapeAlignBranch_t *) 0) {
                    makeTransition = StartTapeBranch(branch, &nextState);
                    ThisEvent.EventType = ES_NO_EVENT;
                }
            } else if (UpdateTapeFlagsFromEvent(ThisEvent.EventType) == TRUE) {
                branch = SelectReadyTapeBranch();
                if (branch != (const TapeAlignBranch_t *) 0) {
                    PostTapeComboEvent(branch);
                }
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        }
        break;

    case TapeTurnLeftState:
    case TapeTurnRightState:
    case TapeStraightLeftState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            AlignSubHSM_UpdateControl();
            break;
        case RealignedEvent:
            RobotMotion_Stop();
            ThisEvent.EventType = RealignedEvent;
            break;
        default:
            if (CompleteIfRealignedTapeOn(&ThisEvent) == TRUE) {
                /* CompleteIfRealignedTapeOn sets ThisEvent to RealignedEvent. */
            } else if ((IsTapeOnEvent(ThisEvent.EventType) == TRUE) ||
                    (IsTapeOffEvent(ThisEvent.EventType) == TRUE)) {
                UpdateTapeFlagsFromEvent(ThisEvent.EventType);
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        }
        break;

    case TapeStraightRightState:
        switch (ThisEvent.EventType) {
        case ES_ENTRY:
            ES_Timer_InitTimer(NAV_SETTLE_TIMER, TAPE_ALIGN_SWEEP_TIMER_MS);
            AlignSubHSM_UpdateControl();
            break;
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == NAV_SETTLE_TIMER) {
                nextState = TapeStraightLeftState;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        case RealignedEvent:
            RobotMotion_Stop();
            ThisEvent.EventType = RealignedEvent;
            break;
        default:
            if (CompleteIfRealignedTapeOn(&ThisEvent) == TRUE) {
                /* CompleteIfRealignedTapeOn sets ThisEvent to RealignedEvent. */
            } else if ((IsTapeOnEvent(ThisEvent.EventType) == TRUE) ||
                    (IsTapeOffEvent(ThisEvent.EventType) == TRUE)) {
                UpdateTapeFlagsFromEvent(ThisEvent.EventType);
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;
        }
        break;

    default:
        break;
    }

    if (makeTransition == TRUE) {
        RunAlignSubHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunAlignSubHSM(ENTRY_EVENT);
        RobotMotion_DebugPrintCurrentCommand("entry");
    }

    ES_Tail();
    return ThisEvent;
}

uint8_t AlignSubHSM_IsActive(void)
{
    return (CurrentState != InitPAlignState) ? TRUE : FALSE;
}

uint8_t AlignSubHSM_IsPositionStage(void)
{
    return FALSE;
}

uint8_t AlignSubHSM_IsHeadingStage(void)
{
    return (CurrentState == GyroHeadingAlignState) ? TRUE : FALSE;
}

uint8_t AlignSubHSM_IsTapeStage(void)
{
    return ((CurrentState == TapeWaitOffState) ||
            (CurrentState == TapeTurnLeftState) ||
            (CurrentState == TapeTurnRightState) ||
            (CurrentState == TapeStraightRightState) ||
            (CurrentState == TapeStraightLeftState)) ? TRUE : FALSE;
}

void AlignSubHSM_UpdateControl(void)
{
    float error;

    switch (CurrentState) {
    case GyroHeadingAlignState:
        error = RobotIMU_GetHeadingErrorToZeroDeg();
        if (AbsFloat(error) <= HEADING_THRESHOLD_DEG) {
            RobotMotion_Stop();
        } else if (error > 0.0f) {
            RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
        } else {
            RobotMotion_TurnRightAbout(TURN_PIVOT_CENTER, TURN_SPEED_IPS);
        }
        break;
    case TapeTurnLeftState:
    case TapeStraightLeftState:
        DriveTapeTurn(TRUE);
        break;
    case TapeTurnRightState:
    case TapeStraightRightState:
        DriveTapeTurn(FALSE);
        break;
    default:
        break;
    }
}

uint8_t AlignSubHSM_IsPositionAligned(void)
{
    return FALSE;
}

uint8_t AlignSubHSM_IsHeadingAligned(void)
{
    if (CurrentState != GyroHeadingAlignState) {
        headingAlignedSamples = 0u;
        return FALSE;
    }
    return StableCheck(AbsFloat(RobotIMU_GetHeadingErrorToZeroDeg()) <= HEADING_THRESHOLD_DEG,
            &headingAlignedSamples);
}

const char *AlignSubHSM_GetStateName(void)
{
    return StateNames[CurrentState];
}

MovementAxis_t AlignSubHSM_GetAxis(void)
{
    return alignAxis;
}

AlignMode_t AlignSubHSM_GetMode(void)
{
    return alignMode;
}

float AlignSubHSM_GetXRef(void)
{
    return xRef;
}

float AlignSubHSM_GetYRef(void)
{
    return yRef;
}

float AlignSubHSM_GetPositionError(void)
{
    if (alignAxis == MOVEMENT_AXIS_HORIZONTAL) {
        return yRef - RobotIMU_GetYInches();
    }
    return xRef - RobotIMU_GetXInches();
}

static uint8_t InitAlignCommon(AlignMode_t mode, MovementAxis_t axis,
        float xRefInches, float yRefInches)
{
    ES_Event returnEvent;

    alignMode = mode;
    alignAxis = axis;
    xRef = xRefInches;
    yRef = yRefInches;
    CurrentState = InitPAlignState;
    headingAlignedSamples = 0u;
    tapeOffFlags = 0u;

    returnEvent = RunAlignSubHSM(INIT_EVENT);
    return (returnEvent.EventType == ES_NO_EVENT) ? TRUE : FALSE;
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t StableCheck(uint8_t inThreshold, uint8_t *sampleCounter)
{
    if (inThreshold == FALSE) {
        *sampleCounter = 0u;
        return FALSE;
    }

    if (*sampleCounter < ALIGN_STABLE_SAMPLE_COUNT) {
        (*sampleCounter)++;
    }

    return (*sampleCounter >= ALIGN_STABLE_SAMPLE_COUNT) ? TRUE : FALSE;
}

static uint8_t TapeMaskForSensor(uint8_t sensorNumber)
{
    if ((sensorNumber < 1u) || (sensorNumber > 5u)) {
        return 0u;
    }
    return (uint8_t) (1u << (sensorNumber - 1u));
}

static uint8_t TapeStateMask(void)
{
    uint8_t mask = 0u;
    uint8_t sensor;

    for (sensor = 1u; sensor <= 5u; sensor++) {
        if (RobotSensors_IsTapeOn((TapeSensor_t) sensor) == TRUE) {
            mask |= TapeMaskForSensor(sensor);
        }
    }
    return mask;
}

static uint8_t TapeOffMaskForBranch(const TapeAlignBranch_t *branch)
{
    return (uint8_t) (TapeMaskForSensor(branch->sensorA) |
            TapeMaskForSensor(branch->sensorB));
}

static const TapeAlignBranch_t *SelectReadyTapeBranch(void)
{
    const TapeAlignBranch_t *branches;
    uint8_t count;
    uint8_t i;

    if (alignAxis == MOVEMENT_AXIS_VERTICAL) {
        branches = VerticalBranches;
        count = (uint8_t) (sizeof(VerticalBranches) / sizeof(VerticalBranches[0]));
    } else {
        branches = HorizontalBranches;
        count = (uint8_t) (sizeof(HorizontalBranches) / sizeof(HorizontalBranches[0]));
    }

    for (i = 0u; i < count; i++) {
        uint8_t pairMask = TapeOffMaskForBranch(&branches[i]);
        if ((tapeOffFlags & pairMask) == pairMask) {
            return &branches[i];
        }
    }

    return (const TapeAlignBranch_t *) 0;
}

static const TapeAlignBranch_t *FindTapeBranchForEvent(ES_EventTyp_t eventType)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t) (sizeof(VerticalBranches) / sizeof(VerticalBranches[0])); i++) {
        if (VerticalBranches[i].comboEvent == eventType) {
            return &VerticalBranches[i];
        }
    }
    for (i = 0u; i < (uint8_t) (sizeof(HorizontalBranches) / sizeof(HorizontalBranches[0])); i++) {
        if (HorizontalBranches[i].comboEvent == eventType) {
            return &HorizontalBranches[i];
        }
    }

    return (const TapeAlignBranch_t *) 0;
}

static uint8_t StartTapeBranch(const TapeAlignBranch_t *branch,
        AlignState_t *nextState)
{
    float headingError;

    activeTapeBranch = *branch;
    *nextState = TapeTurnStateForHeading();
#if defined(DEBUG) || defined(ROBOT_DEBUG)
    headingError = RobotIMU_GetHeadingErrorToZeroDeg();
    printf("[ALIGN] branch %s sensors=%u+%u off pivot=tape%u headingError=",
            EventNames[branch->comboEvent],
            (unsigned int) branch->sensorA,
            (unsigned int) branch->sensorB,
            (unsigned int) branch->pivotSensor);
    PrintFixedDeg(headingError);
    printf(" -> %s\r\n", StateNames[*nextState]);
#else
    (void) headingError;
#endif
    return TRUE;
}

static AlignState_t TapeTurnStateForHeading(void)
{
    float error = RobotIMU_GetHeadingErrorToZeroDeg();

    if (error > TAPE_ALIGN_HEADING_STRAIGHT_DEG) {
        return TapeTurnLeftState;
    }
    if (error < -TAPE_ALIGN_HEADING_STRAIGHT_DEG) {
        return TapeTurnRightState;
    }
    return TapeStraightRightState;
}

static void RefreshTapeOffFlagsFromHardware(void)
{
    uint8_t sensor;

    tapeOffFlags = 0u;
    for (sensor = 1u; sensor <= 5u; sensor++) {
        if ((RobotPlugPlay_IsTapeEnabled(sensor) == TRUE) &&
                (RobotSensors_IsTapeOn((TapeSensor_t) sensor) == FALSE)) {
            tapeOffFlags |= TapeMaskForSensor(sensor);
        }
    }
}

static uint8_t UpdateTapeFlagsFromEvent(ES_EventTyp_t eventType)
{
    uint8_t sensorNumber;
    uint8_t mask;

    if (SensorFromTapeEvent(eventType, &sensorNumber) == FALSE) {
        return FALSE;
    }

    mask = TapeMaskForSensor(sensorNumber);
    if (IsTapeOffEvent(eventType) == TRUE) {
        tapeOffFlags |= mask;
    } else {
        tapeOffFlags &= (uint8_t) ~mask;
    }
    return TRUE;
}

static uint8_t SensorFromTapeEvent(ES_EventTyp_t eventType, uint8_t *sensorNumber)
{
    switch (eventType) {
    case TapeSensor1OnEvent:
    case TapeSensor1OffEvent:
        *sensorNumber = 1u;
        return TRUE;
    case TapeSensor2OnEvent:
    case TapeSensor2OffEvent:
        *sensorNumber = 2u;
        return TRUE;
    case TapeSensor3OnEvent:
    case TapeSensor3OffEvent:
        *sensorNumber = 3u;
        return TRUE;
    case TapeSensor4OnEvent:
    case TapeSensor4OffEvent:
        *sensorNumber = 4u;
        return TRUE;
    case TapeSensor5OnEvent:
    case TapeSensor5OffEvent:
    case TapeSensor5LowToHighEvent:
        *sensorNumber = 5u;
        return TRUE;
    default:
        return FALSE;
    }
}

static uint8_t IsTapeOnEvent(ES_EventTyp_t eventType)
{
    switch (eventType) {
    case TapeSensor1OnEvent:
    case TapeSensor2OnEvent:
    case TapeSensor3OnEvent:
    case TapeSensor4OnEvent:
    case TapeSensor5OnEvent:
    case TapeSensor5LowToHighEvent:
        return TRUE;
    default:
        return FALSE;
    }
}

static uint8_t IsTapeOffEvent(ES_EventTyp_t eventType)
{
    switch (eventType) {
    case TapeSensor1OffEvent:
    case TapeSensor2OffEvent:
    case TapeSensor3OffEvent:
    case TapeSensor4OffEvent:
    case TapeSensor5OffEvent:
        return TRUE;
    default:
        return FALSE;
    }
}

static uint8_t IsTapeComboEvent(ES_EventTyp_t eventType)
{
    switch (eventType) {
    case TapeSensor1And2OffEvent:
    case TapeSensor1And5OffEvent:
    case TapeSensor2And5OffEvent:
    case TapeSensor3And4OffEvent:
    case TapeSensor3And5OffEvent:
    case TapeSensor4And5OffEvent:
        return TRUE;
    default:
        return FALSE;
    }
}

static uint8_t CompleteIfRealignedTapeOn(ES_Event *event)
{
    uint8_t sensorNumber;
    uint8_t sensorMask;

    if (IsTapeOnEvent(event->EventType) == FALSE) {
        return FALSE;
    }
    if (SensorFromTapeEvent(event->EventType, &sensorNumber) == FALSE) {
        return FALSE;
    }

    sensorMask = TapeMaskForSensor(sensorNumber);
    UpdateTapeFlagsFromEvent(event->EventType);
    if ((activeTapeBranch.realignMask & sensorMask) == 0u) {
        event->EventType = ES_NO_EVENT;
        return FALSE;
    }

    RobotMotion_Stop();
    event->EventType = RealignedEvent;
    event->EventParam = ALIGN_REALIGNED_SOURCE_SENSOR;
    return TRUE;
}

static TurnPivot_t PivotForTapeSensor(uint8_t sensorNumber)
{
    switch (sensorNumber) {
    case 1u:
        return TURN_PIVOT_FRONT_CENTER;
    case 2u:
        return TURN_PIVOT_BACK_CENTER;
    case 3u:
        return TURN_PIVOT_LEFT_CENTER;
    case 4u:
        return TURN_PIVOT_RIGHT_CENTER;
    case 5u:
    default:
        return TURN_PIVOT_CENTER;
    }
}

static void DriveTapeTurn(uint8_t turnLeft)
{
    TurnPivot_t pivot = PivotForTapeSensor(activeTapeBranch.pivotSensor);

    if (turnLeft == TRUE) {
        RobotMotion_TurnLeftAbout(pivot, TURN_SPEED_IPS);
    } else {
        RobotMotion_TurnRightAbout(pivot, TURN_SPEED_IPS);
    }
}

static void PostTapeComboEvent(const TapeAlignBranch_t *branch)
{
    ES_Event event;
    uint8_t tapeMask = TapeStateMask();

    event.EventType = branch->comboEvent;
    event.EventParam = tapeMask;
#if defined(DEBUG) || defined(ROBOT_DEBUG)
    printf("[ALIGN] post %s tapeMask=0x%02X pivot=tape%u headingError=",
            EventNames[branch->comboEvent],
            (unsigned int) tapeMask,
            (unsigned int) branch->pivotSensor);
    PrintFixedDeg(RobotIMU_GetHeadingErrorToZeroDeg());
    printf("\r\n");
#endif
    PostRobotHSM(event);
}

static void PrintFixedDeg(float value)
{
    int scaled;

    if (value < 0.0f) {
        printf("-");
        value = -value;
    }

    scaled = (int) ((value * 100.0f) + 0.5f);
    printf("%d.%02d deg", scaled / 100, scaled % 100);
}
