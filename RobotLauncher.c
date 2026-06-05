#include "RobotLauncher.h"

#include <xc.h>

#include "RC_Servo.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"
#include "RobotStepper.h"
#include "pwm.h"

#include <stdio.h>

typedef struct {
    uint16_t beaconADC;
#if ROBOT_PLUGPLAY_USE_LAUNCHER_SERVO
    uint16_t servoPulseUS;
#endif
    int16_t fullStepTarget;
} BeaconAimPoint_t;

/* Dummy Beacon ADC -> full-step launcher LUT, ordered high ADC to low ADC.
 * [FLAG][#1] Replace these placeholder step targets with measured shots.
 * Servo pulse values are retained only for optional servo fallback builds. */
static const BeaconAimPoint_t BeaconAimLUT[] = {
#if ROBOT_PLUGPLAY_USE_LAUNCHER_SERVO
    {900u, 1800u, 3},
    {700u, 1650u, 8},
    {580u, 1500u, 13},
    {512u, 1350u, 17},
#else
    {555u, 13},
    {545u, 12},
    {542u, 8},
    {528u, 3},
    {512u, 0},
#endif
};

#if ROBOT_PLUGPLAY_USE_LAUNCHER_SERVO
static uint16_t PulseForBeacon(uint16_t beaconADC);
#endif
static int16_t StepForBeacon(uint16_t beaconADC);
static uint8_t AimIndexForBeacon(uint16_t beaconADC);
static int16_t ClampStepTarget(int16_t stepTarget);
static uint8_t EnsurePWMReady(void);

uint8_t RobotLauncher_Init(void)
{
    uint8_t ok = TRUE;

#if ROBOT_PLUGPLAY_USE_SHOOTER_MOTOR
    if (EnsurePWMReady() != TRUE) {
        ok = FALSE;
    }
    if (PWM_AddPins(SHOOTER_MOTOR_PWM_PIN) != SUCCESS) {
        ok = FALSE;
    }
#endif

#if ROBOT_PLUGPLAY_USE_LAUNCHER_SERVO
    if (RC_Init() != SUCCESS) {
        ok = FALSE;
    }
    if (RC_AddPins(LAUNCHER_SERVO_PIN) != SUCCESS) {
        ok = FALSE;
    }
    RC_SetPulseTime(LAUNCHER_SERVO_PIN, LAUNCHER_SERVO_CENTER_PULSE_US);
#endif

    return ok;
}

void RobotLauncher_SetAngleFromBeaconADC(uint16_t beaconADC)
{
#if ROBOT_PLUGPLAY_USE_STEPPER
    RobotStepper_MoveToStep(StepForBeacon(beaconADC));
#elif ROBOT_PLUGPLAY_USE_LAUNCHER_SERVO
    RC_SetPulseTime(LAUNCHER_SERVO_PIN, PulseForBeacon(beaconADC));
#else
    (void) beaconADC;
#endif
}

int16_t RobotLauncher_GetTargetStepForBeaconADC(uint16_t beaconADC)
{
    return StepForBeacon(beaconADC);
}

void RobotLauncher_LogAimLUTSelection(uint16_t beaconADC)
{
    uint8_t selectedIndex = AimIndexForBeacon(beaconADC);

    printf("[SHOOT] LUT[%u] adc=%u step=%d\r\n",
            (unsigned int) selectedIndex,
            (unsigned int) beaconADC,
            (int) BeaconAimLUT[selectedIndex].fullStepTarget);
}

void RobotLauncher_StartShooter(void)
{
#if ROBOT_PLUGPLAY_USE_SHOOTER_MOTOR
    PWM_SetDutyCycle(SHOOTER_MOTOR_PWM_PIN, SHOOTER_MOTOR_DUTY);
#endif
}

void RobotLauncher_StopShooter(void)
{
#if ROBOT_PLUGPLAY_USE_SHOOTER_MOTOR
    PWM_SetDutyCycle(SHOOTER_MOTOR_PWM_PIN, 0u);
#endif
}

static uint8_t EnsurePWMReady(void)
{
    if (PWM_ListPins() != 0u) {
        return TRUE;
    }

    if (PWM_Init() != SUCCESS) {
        return FALSE;
    }
    if (PWM_SetFrequency(PWM_1KHZ) != SUCCESS) {
        return FALSE;
    }
    return TRUE;
}

#if ROBOT_PLUGPLAY_USE_LAUNCHER_SERVO
static uint16_t PulseForBeacon(uint16_t beaconADC)
{
    uint8_t i;

    if (beaconADC <= BeaconAimLUT[0].beaconADC) {
        return BeaconAimLUT[0].servoPulseUS;
    }

    for (i = 1; i < (sizeof (BeaconAimLUT) / sizeof (BeaconAimLUT[0])); i++) {
        if (beaconADC <= BeaconAimLUT[i].beaconADC) {
            uint16_t adc0 = BeaconAimLUT[i - 1u].beaconADC;
            uint16_t adc1 = BeaconAimLUT[i].beaconADC;
            int32_t pulse0 = (int32_t) BeaconAimLUT[i - 1u].servoPulseUS;
            int32_t pulse1 = (int32_t) BeaconAimLUT[i].servoPulseUS;
            int32_t delta = pulse1 - pulse0;
            int32_t scaled = ((int32_t) (beaconADC - adc0) * delta) / (int32_t) (adc1 - adc0);

            return (uint16_t) (pulse0 + scaled);
        }
    }

    return BeaconAimLUT[(sizeof (BeaconAimLUT) / sizeof (BeaconAimLUT[0])) - 1u].servoPulseUS;
}
#endif

static int16_t StepForBeacon(uint16_t beaconADC)
{
    return ClampStepTarget(BeaconAimLUT[AimIndexForBeacon(beaconADC)].fullStepTarget);
}

static uint8_t AimIndexForBeacon(uint16_t beaconADC)
{
    uint8_t i;

    for (i = 0u; i < (sizeof (BeaconAimLUT) / sizeof (BeaconAimLUT[0])); i++) {
        if (beaconADC >= BeaconAimLUT[i].beaconADC) {
            return i;
        }
    }

    return (uint8_t) ((sizeof (BeaconAimLUT) / sizeof (BeaconAimLUT[0])) - 1u);
}

static int16_t ClampStepTarget(int16_t stepTarget)
{
    if (stepTarget < LAUNCHER_STEPPER_MIN_STEP) {
        return LAUNCHER_STEPPER_MIN_STEP;
    }
    if (stepTarget > LAUNCHER_STEPPER_MAX_STEP) {
        return LAUNCHER_STEPPER_MAX_STEP;
    }
    return stepTarget;
}
