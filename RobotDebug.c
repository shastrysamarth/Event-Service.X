#include "RobotDebug.h"

#include "AlignSubHSM.h"
#include "FindFrontTapeSubHSM.h"
#include "NavigateToISZSubHSM.h"
#include "RobotHSM.h"
#include "RobotIMU.h"
#include "RobotMotion.h"
#include "RobotPlugPlay.h"
#include "ShootingSubHSM.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

static const char *BoundaryName(BoundaryChoice_t choice);
static const char *MovementAxisName(MovementAxis_t axis);
static const char *DistanceAxisName(DistanceAxis_t axis);
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
    printf("[VAR] NavigateToISZ.state=%s boundary_choice=%s movement_axis=%s num_tapes_crossed=%u\r\n",
            NavigateToISZ_GetStateName(),
            BoundaryName(NavigateToISZ_GetBoundaryChoice()),
            MovementAxisName(NavigateToISZ_GetMovementAxis()),
            (unsigned int) NavigateToISZ_GetNumTapesCrossed());
    PrintFixedValue("[VAR] NavigateToISZ.x_ref", NavigateToISZ_GetXRef(), "in");
    PrintFixedValue("[VAR] NavigateToISZ.y_ref", NavigateToISZ_GetYRef(), "in");
    printf("[VAR] Align.state=%s alignAxis=%s\r\n",
            AlignSubHSM_GetStateName(),
            MovementAxisName(AlignSubHSM_GetAxis()));
    PrintFixedValue("[VAR] Align.xRef", AlignSubHSM_GetXRef(), "in");
    PrintFixedValue("[VAR] Align.yRef", AlignSubHSM_GetYRef(), "in");
    PrintFixedValue("[VAR] Align.positionError", AlignSubHSM_GetPositionError(), "in");
    PrintFixedValue("[VAR] Align.headingError", RobotIMU_GetHeadingErrorToZeroDeg(), "deg");
    printf("[VAR] Shooting.state=%s maxBeaconADC=%u\r\n",
            ShootingSubHSM_GetStateName(),
            (unsigned int) ShootingSubHSM_GetMaxBeaconADC());
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
    PrintFixedValue("[VAR] IMU.zGyro", RobotIMU_GetZGyroDPS(), "dps");
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
