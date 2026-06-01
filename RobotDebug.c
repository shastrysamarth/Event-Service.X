#include "RobotDebug.h"

#include "AlignSubHSM.h"
#include "FindFrontTapeSubHSM.h"
#include "NavigateToISZSubHSM.h"
#include "RobotHSM.h"
#include "RobotIMU.h"
#include "RobotMotion.h"
#include "RobotPlugPlay.h"
#include "RobotSensors.h"
#include "RobotStepper.h"
#include "RobotLauncher.h"
#include "ShootingSubHSM.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

const char * const EventNames[] = {
    "ES_NO_EVENT",
    "ES_ERROR",
    "ES_INIT",
    "ES_ENTRY",
    "ES_EXIT",
    "ES_KEYINPUT",
    "ES_LISTEVENTS",
    "ES_TIMEOUT",
    "ES_TIMERACTIVE",
    "ES_TIMERSTOPPED",
    "BeaconADCIncreaseEvent",
    "MaxSignalFoundEvent",
    "TapeSensor1OnEvent",
    "TapeSensor1OffEvent",
    "TapeSensor2OnEvent",
    "TapeSensor2OffEvent",
    "TapeSensor3OnEvent",
    "TapeSensor3OffEvent",
    "TapeSensor4OnEvent",
    "TapeSensor4OffEvent",
    "TapeSensor5OnEvent",
    "TapeSensor5OffEvent",
    "TapeSensor5LowToHighEvent",
    "TapeSensor1And2OffEvent",
    "TapeSensor1And5OffEvent",
    "TapeSensor2And5OffEvent",
    "TapeSensor3And4OffEvent",
    "TapeSensor3And5OffEvent",
    "TapeSensor4And5OffEvent",
    "Solenoid1OnEvent",
    "Solenoid2OnEvent",
    "Solenoid3OnEvent",
    "Solenoid4OnEvent",
    "Solenoid5OnEvent",
    "Solenoid6OnEvent",
    "BumpSensor1OnEvent",
    "BumpSensor1OffEvent",
    "BumpSensor2OnEvent",
    "BumpSensor2OffEvent",
    "BumpSensor3OnEvent",
    "BumpSensor3OffEvent",
    "BumpSensor4OnEvent",
    "BumpSensor4OffEvent",
    "MisalignedEvent",
    "PositionRealignedEvent",
    "RealignedEvent",
    "DistanceMoveCompleteEvent",
    "FoundFrontTapeEvent",
    "ReachedISZEvent",
    "InsideISZEvent",
    "SetEvent",
    "DoneEvent",
    "NUMBEROFEVENTS",
};

static const char *BoundaryName(BoundaryChoice_t choice);
static const char *MovementAxisName(MovementAxis_t axis);
static const char *DistanceAxisName(DistanceAxis_t axis);
static const char *AlignModeName(AlignMode_t mode);
static void PrintFixedValue(const char *name, float value, const char *unit);

void RobotDebug_LogStateEntry(const char *machineName,
        const char *stateName, ES_Event event)
{
#if defined(DEBUG) || defined(ROBOT_DEBUG)
    if (event.EventType == ES_ENTRY) {
        printf("[STATE] %s.%s\r\n", machineName, stateName);
    }
#else
    (void) machineName;
    (void) stateName;
    (void) event;
#endif
}

void RobotDebug_PrintCurrentState(void)
{
#if defined(DEBUG) || defined(ROBOT_DEBUG)
    const char *topState = RobotHSM_GetStateName();

    printf("[STATE] %s", topState);

    if (strcmp(topState, "FindFrontTapeState") == 0) {
        printf(" > %s", FindFrontTape_GetStateName());
    } else if (strcmp(topState, "NavigateToISZState") == 0) {
        printf(" > %s", NavigateToISZ_GetStateName());
        if (NavigateToISZ_IsAligning() == TRUE) {
            printf(" > %s", AlignSubHSM_GetStateName());
        }
    } else if (strcmp(topState, "ShootState") == 0) {
        printf(" > %s", ShootingSubHSM_GetStateName());
        if (ShootingSubHSM_IsAligning() == TRUE) {
            printf(" > %s", AlignSubHSM_GetStateName());
        }
    }

    printf("\r\n");
#endif
}

void RobotDebug_PrintModuleVariables(void)
{
#if defined(DEBUG) || defined(ROBOT_DEBUG)
    uint8_t imuReady = RobotIMU_IsReady();
    uint8_t imuCalibrated = imuReady ? RobotIMU_IsFullyCalibrated() : FALSE;

    printf("\r\n[VAR] Module variables\r\n");
    printf("[VAR] RobotHSM.state=%s\r\n", RobotHSM_GetStateName());
    printf("[VAR] FindFrontTape.state=%s boundary_choice=%s\r\n",
            FindFrontTape_GetStateName(),
            BoundaryName(FindFrontTape_GetBoundaryChoice()));
    printf("[VAR] NavigateToISZ.state=%s\r\n", NavigateToISZ_GetStateName());
    printf("[VAR] NavigateToISZ.boundary=%s axis=%s tapes=%u\r\n",
            BoundaryName(NavigateToISZ_GetBoundaryChoice()),
            MovementAxisName(NavigateToISZ_GetMovementAxis()),
            (unsigned int) NavigateToISZ_GetNumTapesCrossed());
    printf("[VAR] Align.state=%s mode=%s axis=%s\r\n",
            AlignSubHSM_GetStateName(),
            AlignModeName(AlignSubHSM_GetMode()),
            MovementAxisName(AlignSubHSM_GetAxis()));
    PrintFixedValue("[VAR] Align.headingError", RobotIMU_GetHeadingErrorToZeroDeg(), "deg");
    printf("[VAR] Shooting.state=%s maxBeaconADC=%u targetStep=%d currentStep=%d\r\n",
            ShootingSubHSM_GetStateName(),
            (unsigned int) ShootingSubHSM_GetMaxBeaconADC(),
            (int) RobotLauncher_GetTargetStepForBeaconADC(ShootingSubHSM_GetMaxBeaconADC()),
            (int) RobotStepper_GetPosition());
    {
        uint16_t beaconRaw = RobotSensors_ReadBeaconRawADC();
        uint16_t beaconSmooth = RobotSensors_ReadBeaconADC();
        printf("[VAR] BeaconADC.enabled=%u raw=%u smooth=%u distance=%u ft\r\n",
                (unsigned int) ROBOT_PLUGPLAY_USE_BEACON_ADC,
                (unsigned int) beaconRaw,
                (unsigned int) beaconSmooth,
                (unsigned int) RobotSensors_BeaconDistanceFeetFromADC(beaconSmooth));
    }
    printf("[VAR] BNO055.enabled=%u ready=%u calibrated=%u\r\n",
            (unsigned int) ROBOT_PLUGPLAY_USE_BNO055,
            (unsigned int) imuReady,
            (unsigned int) imuCalibrated);
    printf("[VAR] IMU.stationary=%u\r\n", (unsigned int) RobotIMU_IsStationary());
    PrintFixedValue("[VAR] IMU.heading", RobotIMU_GetHeadingDeg(), "deg");
    PrintFixedValue("[VAR] IMU.headingOffset", RobotIMU_GetHeadingOffsetDeg(), "deg");
    PrintFixedValue("[VAR] IMU.headingErrorToZero", RobotIMU_GetHeadingErrorToZeroDeg(), "deg");
    PrintFixedValue("[VAR] IMU.x", RobotIMU_GetXInches(), "in");
    PrintFixedValue("[VAR] IMU.y", RobotIMU_GetYInches(), "in");
    PrintFixedValue("[VAR] IMU.xAccel", RobotIMU_GetXAccelIPS2(), "in/s^2");
    PrintFixedValue("[VAR] IMU.yAccel", RobotIMU_GetYAccelIPS2(), "in/s^2");
    PrintFixedValue("[VAR] IMU.xVelocity", RobotIMU_GetXVelocityIPS(), "in/s");
    PrintFixedValue("[VAR] IMU.yVelocity", RobotIMU_GetYVelocityIPS(), "in/s");
    PrintFixedValue("[VAR] IMU.headingGyro", RobotIMU_GetZGyroDPS(), "dps");
    printf("[VAR] Motion.distanceMoveActive=%u axis=%s direction=%d\r\n",
            (unsigned int) RobotMotion_IsDistanceMoveActive(),
            DistanceAxisName(RobotMotion_GetDistanceAxis()),
            (int) RobotMotion_GetDistanceDirection());
    PrintFixedValue("[VAR] Motion.distanceTarget", RobotMotion_GetDistanceTargetInches(), "in");
    printf("\r\n");
#endif
}

static const char *BoundaryName(BoundaryChoice_t choice)
{
    return (choice == BOUNDARY_TOP) ? "TOP" : "BOTTOM";
}

static const char *MovementAxisName(MovementAxis_t axis)
{
    return (axis == MOVEMENT_AXIS_HORIZONTAL) ? "HORIZONTAL" : "VERTICAL";
}

static const char *DistanceAxisName(DistanceAxis_t axis)
{
    return (axis == DISTANCE_AXIS_X) ? "X" : "Y";
}

static const char *AlignModeName(AlignMode_t mode)
{
    return (mode == ALIGN_MODE_TAPE) ? "TAPE" : "GYRO";
}

static void PrintFixedValue(const char *name, float value, const char *unit)
{
    int32_t scaled = (int32_t) (value * 100.0f);
    uint32_t magnitude;

    if (scaled < 0) {
        magnitude = (uint32_t) (-scaled);
        printf("%s=-%lu.%02lu %s\r\n", name,
                (unsigned long) (magnitude / 100u),
                (unsigned long) (magnitude % 100u),
                unit);
    } else {
        magnitude = (uint32_t) scaled;
        printf("%s=%lu.%02lu %s\r\n", name,
                (unsigned long) (magnitude / 100u),
                (unsigned long) (magnitude % 100u),
                unit);
    }
}
