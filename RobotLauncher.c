#include "RobotLauncher.h"

#include <xc.h>

#include "RC_Servo.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"
#include "pwm.h"

typedef struct {
    uint16_t beaconADC;
    uint16_t servoPulseUS;
} BeaconAimPoint_t;

/* [FLAG][#1] Fill this LUT from beacon ADC calibration tests. */
static const BeaconAimPoint_t BeaconAimLUT[] = {
    {100u, 1850u},
    {300u, 1700u},
    {500u, 1550u},
    {700u, 1400u},
    {900u, 1250u},
};

static uint16_t PulseForBeacon(uint16_t beaconADC);
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
#if ROBOT_PLUGPLAY_USE_LAUNCHER_SERVO
    RC_SetPulseTime(LAUNCHER_SERVO_PIN, PulseForBeacon(beaconADC));
#else
    (void) beaconADC;
#endif
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
