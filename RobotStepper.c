#include "RobotStepper.h"

#include <xc.h>

#include "RobotPins.h"
#include "RobotPlugPlay.h"

uint8_t RobotStepper_Init(void)
{
#if ROBOT_PLUGPLAY_USE_STEPPER
    STEPPER_STEP_TRIS = 0;
    STEPPER_DIR_TRIS = 0;
    STEPPER_ENABLE_TRIS = 0;
    RobotStepper_Disable();
#endif
    return TRUE;
}

void RobotStepper_Enable(void)
{
#if ROBOT_PLUGPLAY_USE_STEPPER
#if STEPPER_ENABLE_ACTIVE_LOW
    STEPPER_ENABLE_LAT = 0;
#else
    STEPPER_ENABLE_LAT = 1;
#endif
#endif
}

void RobotStepper_Disable(void)
{
#if ROBOT_PLUGPLAY_USE_STEPPER
    STEPPER_STEP_LAT = 0;
#if STEPPER_ENABLE_ACTIVE_LOW
    STEPPER_ENABLE_LAT = 1;
#else
    STEPPER_ENABLE_LAT = 0;
#endif
#endif
}

void RobotStepper_SetDirection(uint8_t forward)
{
#if ROBOT_PLUGPLAY_USE_STEPPER
    STEPPER_DIR_LAT = forward ? 1 : 0;
#else
    (void) forward;
#endif
}

void RobotStepper_PulseStep(void)
{
#if ROBOT_PLUGPLAY_USE_STEPPER
    STEPPER_STEP_LAT = 1;
    STEPPER_STEP_LAT = 0;
#endif
}
