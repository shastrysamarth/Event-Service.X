#include "RobotPlugPlay.h"

#include <stdio.h>

#include "RobotPins.h"

typedef struct {
    uint8_t enabled;
    const char *name;
    const char *pinLabel;
} PinoutLine_t;

static void PrintLine(uint8_t enabled, const char *name, const char *pinLabel);

void RobotPlugPlay_PrintConfig(void)
{
    static const PinoutLine_t lines[] = {
        {ROBOT_PLUGPLAY_USE_BNO055, "BNO055 gyro/accel", BNO055_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_BEACON_ADC, "Beacon ADC", BEACON_ADC_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_TAPE1, "Tape sensor 1", TAPE_SENSOR_1_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_TAPE2, "Tape sensor 2", TAPE_SENSOR_2_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_TAPE3, "Tape sensor 3", TAPE_SENSOR_3_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_TAPE4, "Tape sensor 4", TAPE_SENSOR_4_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_TAPE5, "Tape sensor 5", TAPE_SENSOR_5_PIN_LABEL},
        /* Solenoid hardware placement intentionally disabled for now. */
        {ROBOT_PLUGPLAY_USE_BUMP1, "Bump sensor 1", BUMP_SENSOR_1_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_BUMP2, "Bump sensor 2", BUMP_SENSOR_2_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_BUMP3, "Bump sensor 3", BUMP_SENSOR_3_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_BUMP4, "Bump sensor 4", BUMP_SENSOR_4_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_DRIVE_MOTORS, "Mecanum drive motors", "FL: " MOTOR_FL_PIN_LABEL ", FR: " MOTOR_FR_PIN_LABEL ", RL: " MOTOR_RL_PIN_LABEL ", RR: " MOTOR_RR_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_LAUNCHER_SERVO, "Launcher angle servo", LAUNCHER_SERVO_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_SHOOTER_MOTOR, "Shooter motor", SHOOTER_MOTOR_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_SHOOTER_ADC, "Shooter ADC input", SHOOTER_MOTOR_ADC_PIN_LABEL},
        {ROBOT_PLUGPLAY_USE_STEPPER, "Stepper driver", STEPPER_PIN_LABEL},
    };
    uint8_t i;

    printf("\r\n[HW] Plug-and-play hardware configuration\r\n");
#ifdef ROBOT_MOTOR_SENSOR_TEST
    printf("[HW] Motor/sensor bench mode: HSM is disabled; direct keyboard controls are active.\r\n");
#elif defined(ROBOT_BEACON_TEST)
    printf("[HW] Beacon bench mode: HSM is disabled; V8 ADC smoothing is active.\r\n");
#elif defined(ROBOT_GPIO_HIGH_TEST)
    printf("[HW] GPIO-high bench mode: HSM is disabled; connector pins are driven high.\r\n");
#elif defined(ROBOT_KEYBOARD_TEST)
    printf("[HW] Test mode: disabled hardware is ignored; keyboard events still work.\r\n");
#else
    printf("[HW] Robot mode: hardware groups default enabled.\r\n");
#endif
    for (i = 0u; i < (sizeof(lines) / sizeof(lines[0])); i++) {
        PrintLine(lines[i].enabled, lines[i].name, lines[i].pinLabel);
    }
    printf("\r\n");
}

uint8_t RobotPlugPlay_IsTapeEnabled(uint8_t sensorNumber)
{
    switch (sensorNumber) {
    case 1:
        return ROBOT_PLUGPLAY_USE_TAPE1;
    case 2:
        return ROBOT_PLUGPLAY_USE_TAPE2;
    case 3:
        return ROBOT_PLUGPLAY_USE_TAPE3;
    case 4:
        return ROBOT_PLUGPLAY_USE_TAPE4;
    case 5:
        return ROBOT_PLUGPLAY_USE_TAPE5;
    default:
        return FALSE;
    }
}

uint8_t RobotPlugPlay_IsSolenoidADCEnabled(uint8_t sensorNumber)
{
    switch (sensorNumber) {
    case 1:
        return ROBOT_PLUGPLAY_USE_SOLENOID1_ADC;
    case 2:
        return ROBOT_PLUGPLAY_USE_SOLENOID2_ADC;
    case 3:
        return ROBOT_PLUGPLAY_USE_SOLENOID3_ADC;
    case 4:
        return ROBOT_PLUGPLAY_USE_SOLENOID4_ADC;
    case 5:
        return ROBOT_PLUGPLAY_USE_SOLENOID5_ADC;
    case 6:
        return ROBOT_PLUGPLAY_USE_SOLENOID6_ADC;
    default:
        return FALSE;
    }
}

uint8_t RobotPlugPlay_IsBumpEnabled(uint8_t sensorNumber)
{
    switch (sensorNumber) {
    case 1:
        return ROBOT_PLUGPLAY_USE_BUMP1;
    case 2:
        return ROBOT_PLUGPLAY_USE_BUMP2;
    case 3:
        return ROBOT_PLUGPLAY_USE_BUMP3;
    case 4:
        return ROBOT_PLUGPLAY_USE_BUMP4;
    default:
        return FALSE;
    }
}

static void PrintLine(uint8_t enabled, const char *name, const char *pinLabel)
{
    printf("[HW] %-22s : %s", name, enabled ? "ENABLED " : "disabled");
    if (enabled) {
        printf(" -> %s", pinLabel);
    }
    printf("\r\n");
}
