#include "RobotStepper.h"

#include <xc.h>

#include "RobotPins.h"
#include "RobotPlugPlay.h"

static void StepPulseDelay(void);
static int16_t currentStep = 0;

uint8_t RobotStepper_Init(void)
{
    /* Hold STEP/DIR quiet even when the stepper feature is compiled off.
     * The driver may still be physically connected during drive-only tests. */
    STEPPER_STEP_LAT = 0;
    STEPPER_DIR_LAT = 0;
    STEPPER_STEP_TRIS = 0;
    STEPPER_DIR_TRIS = 0;
#if ROBOT_PLUGPLAY_USE_STEPPER && STEPPER_HAS_ENABLE_PIN
    STEPPER_ENABLE_TRIS = 0;
#endif
    RobotStepper_Disable();
#if ROBOT_PLUGPLAY_USE_STEPPER
    currentStep = LAUNCHER_STEPPER_HOME_STEP;
#endif
    return TRUE;
}

void RobotStepper_Enable(void)
{
#if ROBOT_PLUGPLAY_USE_STEPPER && STEPPER_HAS_ENABLE_PIN
#if STEPPER_ENABLE_ACTIVE_LOW
    STEPPER_ENABLE_LAT = 0;
#else
    STEPPER_ENABLE_LAT = 1;
#endif
#endif
}

void RobotStepper_Disable(void)
{
    STEPPER_STEP_LAT = 0;
    STEPPER_DIR_LAT = 0;
#if ROBOT_PLUGPLAY_USE_STEPPER && STEPPER_HAS_ENABLE_PIN
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
    StepPulseDelay();
    STEPPER_STEP_LAT = 0;
    StepPulseDelay();
#endif
}

void RobotStepper_Step(uint16_t steps, uint8_t forward)
{
#if ROBOT_PLUGPLAY_USE_STEPPER
    uint16_t i;

    RobotStepper_Enable();
    RobotStepper_SetDirection(forward);
    for (i = 0u; i < steps; i++) {
        RobotStepper_PulseStep();
        currentStep += forward ? 1 : -1;
    }
#else
    (void) steps;
    (void) forward;
#endif
}

void RobotStepper_MoveToStep(int16_t targetStep)
{
#if ROBOT_PLUGPLAY_USE_STEPPER
    int16_t delta = targetStep - currentStep;

    RobotStepper_Enable();
    if (delta > 0) {
        RobotStepper_Step((uint16_t) delta, TRUE);
    } else if (delta < 0) {
        RobotStepper_Step((uint16_t) -delta, FALSE);
    }
#else
    (void) targetStep;
#endif
}

void RobotStepper_ZeroPosition(void)
{
    currentStep = LAUNCHER_STEPPER_HOME_STEP;
}

int16_t RobotStepper_GetPosition(void)
{
    return currentStep;
}

static void StepPulseDelay(void)
{
    volatile uint16_t i;

    for (i = 0u; i < 200u; i++) {
        __asm__("nop");
    }
}
