#include "RobotMotion.h"

#include <stdlib.h>
#include <xc.h>

#include "ES_Timers.h"
#include "IO_Ports.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"
#include "pwm.h"

typedef enum {
    MOTOR_FRONT_LEFT,
    MOTOR_FRONT_RIGHT,
    MOTOR_REAR_LEFT,
    MOTOR_REAR_RIGHT,
} DriveMotor_t;

static uint8_t distanceMoveActive = FALSE;
static DistanceAxis_t distanceAxis = DISTANCE_AXIS_Y;
static int8_t distanceDirection = 1;
/* DEPRECATED for open-loop timing: was IMU odometry target; kept for debug getters. */
static float distanceTargetInches = 0.0f;
/* Open-loop distance: elapsed ms vs (inches / MOTOR_SPEED_IPS) * 1000 */
static uint32_t distanceStartMs = 0u;
static uint32_t distanceDurationMs = 0u;

static void ConfigureDirectionPins(void);
static uint8_t EnsurePWMReady(void);
static void SetChassisVelocity(float vxIPS, float vyIPS, float omegaRadPerSec);
static void SetDriveWheels(float frontLeft, float frontRight, float rearLeft, float rearRight);
static void SetMotor(DriveMotor_t motor, int speedDuty);
static void SetMotorDirection(DriveMotor_t motor, int forward);
static int SpeedToDuty(float wheelSpeedIPS);
static float AbsFloat(float value);
static void PivotOffset(TurnPivot_t pivot, float *xOffsetIn, float *yOffsetIn);

uint8_t RobotMotion_Init(void)
{
    uint8_t ok = TRUE;

#if ROBOT_PLUGPLAY_USE_DRIVE_MOTORS
    if (EnsurePWMReady() != TRUE) {
        ok = FALSE;
    }
    if (PWM_AddPins(DRIVE_MOTOR_PWM_PINS) != SUCCESS) {
        ok = FALSE;
    }

    ConfigureDirectionPins();
    RobotMotion_Stop();
#endif
    return ok;
}

void RobotMotion_Stop(void)
{
    SetDriveWheels(0.0f, 0.0f, 0.0f, 0.0f);
}

void RobotMotion_Forward(float speedIPS)
{
    SetChassisVelocity(0.0f, speedIPS, 0.0f);
}

void RobotMotion_Reverse(float speedIPS)
{
    SetChassisVelocity(0.0f, -speedIPS, 0.0f);
}

void RobotMotion_StrafeRight(float speedIPS)
{
    SetChassisVelocity(speedIPS, 0.0f, 0.0f);
}

void RobotMotion_StrafeLeft(float speedIPS)
{
    SetChassisVelocity(-speedIPS, 0.0f, 0.0f);
}

void RobotMotion_TurnLeftAbout(TurnPivot_t pivot, float speedIPS)
{
    float xOffset;
    float yOffset;
    float omega;

    PivotOffset(pivot, &xOffset, &yOffset);
    omega = speedIPS / (ROBOT_HALF_WIDTH_IN + ROBOT_HALF_LENGTH_IN);
    SetChassisVelocity(omega * yOffset, -omega * xOffset, omega);
}

void RobotMotion_TurnRightAbout(TurnPivot_t pivot, float speedIPS)
{
    float xOffset;
    float yOffset;
    float omega;

    PivotOffset(pivot, &xOffset, &yOffset);
    omega = -speedIPS / (ROBOT_HALF_WIDTH_IN + ROBOT_HALF_LENGTH_IN);
    SetChassisVelocity(omega * yOffset, -omega * xOffset, omega);
}

void RobotMotion_StartDistanceMove(DistanceAxis_t axis, int8_t direction, float targetInches)
{
    float durationFloat;

    distanceMoveActive = TRUE;
    distanceAxis = axis;
    distanceDirection = (direction >= 0) ? 1 : -1;
    distanceTargetInches = targetInches;

    /* Chassis translation time from nominal MOTOR_SPEED_IPS (same as Forward/Reverse in nav). */
    durationFloat = (targetInches / MOTOR_SPEED_IPS) * 1000.0f;
    if (durationFloat < 1.0f) {
        durationFloat = 1.0f;
    }
    distanceDurationMs = (uint32_t) (durationFloat + 0.5f);
    distanceStartMs = ES_Timer_GetTime();
}

void RobotMotion_StopDistanceMove(void)
{
    distanceMoveActive = FALSE;
}

uint8_t RobotMotion_IsDistanceMoveActive(void)
{
    return distanceMoveActive;
}

uint8_t RobotMotion_IsDistanceMoveComplete(void)
{
    uint32_t now;
    uint32_t elapsed;

    if (distanceMoveActive == FALSE) {
        return FALSE;
    }

    now = ES_Timer_GetTime();
    elapsed = now - distanceStartMs;
    if (elapsed >= distanceDurationMs) {
        distanceMoveActive = FALSE;
        return TRUE;
    }

    return FALSE;
}

DistanceAxis_t RobotMotion_GetDistanceAxis(void)
{
    return distanceAxis;
}

int8_t RobotMotion_GetDistanceDirection(void)
{
    return distanceDirection;
}

float RobotMotion_GetDistanceTargetInches(void)
{
    return distanceTargetInches;
}

static void ConfigureDirectionPins(void)
{
#if ROBOT_PLUGPLAY_USE_DRIVE_MOTORS
    MOTOR_FL_IN1_TRIS = 0;
    MOTOR_FL_IN2_TRIS = 0;
    MOTOR_FR_IN1_TRIS = 0;
    MOTOR_FR_IN2_TRIS = 0;
    MOTOR_RL_IN1_TRIS = 0;
    MOTOR_RL_IN2_TRIS = 0;
    MOTOR_RR_IN1_TRIS = 0;
    MOTOR_RR_IN2_TRIS = 0;
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

static void SetChassisVelocity(float vxIPS, float vyIPS, float omegaRadPerSec)
{
    float radiusSum = ROBOT_HALF_WIDTH_IN + ROBOT_HALF_LENGTH_IN;
    float frontLeft = vyIPS + vxIPS - (omegaRadPerSec * radiusSum);
    float frontRight = vyIPS - vxIPS + (omegaRadPerSec * radiusSum);
    float rearLeft = vyIPS - vxIPS - (omegaRadPerSec * radiusSum);
    float rearRight = vyIPS + vxIPS + (omegaRadPerSec * radiusSum);

    SetDriveWheels(frontLeft, frontRight, rearLeft, rearRight);
}

static void SetDriveWheels(float frontLeft, float frontRight, float rearLeft, float rearRight)
{
    SetMotor(MOTOR_FRONT_LEFT, SpeedToDuty(frontLeft) * MOTOR_FL_POLARITY);
    SetMotor(MOTOR_FRONT_RIGHT, SpeedToDuty(frontRight) * MOTOR_FR_POLARITY);
    SetMotor(MOTOR_REAR_LEFT, SpeedToDuty(rearLeft) * MOTOR_RL_POLARITY);
    SetMotor(MOTOR_REAR_RIGHT, SpeedToDuty(rearRight) * MOTOR_RR_POLARITY);
}

static void SetMotor(DriveMotor_t motor, int speedDuty)
{
#if ROBOT_PLUGPLAY_USE_DRIVE_MOTORS
    uint16_t duty = (uint16_t) abs(speedDuty);
    int forward = (speedDuty >= 0);

    if (duty > MAX_PWM) {
        duty = MAX_PWM;
    }

    SetMotorDirection(motor, forward);

    switch (motor) {
    case MOTOR_FRONT_LEFT:
        PWM_SetDutyCycle(MOTOR_FL_PWM_PIN, duty);
        break;
    case MOTOR_FRONT_RIGHT:
        PWM_SetDutyCycle(MOTOR_FR_PWM_PIN, duty);
        break;
    case MOTOR_REAR_LEFT:
        PWM_SetDutyCycle(MOTOR_RL_PWM_PIN, duty);
        break;
    case MOTOR_REAR_RIGHT:
        PWM_SetDutyCycle(MOTOR_RR_PWM_PIN, duty);
        break;
    default:
        break;
    }
#else
    (void) motor;
    (void) speedDuty;
#endif
}

static void SetMotorDirection(DriveMotor_t motor, int forward)
{
#if ROBOT_PLUGPLAY_USE_DRIVE_MOTORS
    switch (motor) {
    case MOTOR_FRONT_LEFT:
        MOTOR_FL_IN1_LAT = forward ? 1 : 0;
        MOTOR_FL_IN2_LAT = forward ? 0 : 1;
        break;
    case MOTOR_FRONT_RIGHT:
        MOTOR_FR_IN1_LAT = forward ? 1 : 0;
        MOTOR_FR_IN2_LAT = forward ? 0 : 1;
        break;
    case MOTOR_REAR_LEFT:
        MOTOR_RL_IN1_LAT = forward ? 1 : 0;
        MOTOR_RL_IN2_LAT = forward ? 0 : 1;
        break;
    case MOTOR_REAR_RIGHT:
        MOTOR_RR_IN1_LAT = forward ? 1 : 0;
        MOTOR_RR_IN2_LAT = forward ? 0 : 1;
        break;
    default:
        break;
    }
#else
    (void) motor;
    (void) forward;
#endif
}

static int SpeedToDuty(float wheelSpeedIPS)
{
    float magnitude = AbsFloat(wheelSpeedIPS);
    int duty = (int) (magnitude * MOTOR_DUTY_PER_IPS);

    if (duty > MAX_PWM) {
        duty = MAX_PWM;
    }
    if (wheelSpeedIPS < 0.0f) {
        duty = -duty;
    }
    return duty;
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void PivotOffset(TurnPivot_t pivot, float *xOffsetIn, float *yOffsetIn)
{
    *xOffsetIn = 0.0f;
    *yOffsetIn = 0.0f;

    switch (pivot) {
    case TURN_PIVOT_FRONT_CENTER:
        *yOffsetIn = ROBOT_HALF_LENGTH_IN;
        break;
    case TURN_PIVOT_BACK_CENTER:
        *yOffsetIn = -ROBOT_HALF_LENGTH_IN;
        break;
    case TURN_PIVOT_LEFT_CENTER:
        *xOffsetIn = -ROBOT_HALF_WIDTH_IN;
        break;
    case TURN_PIVOT_RIGHT_CENTER:
        *xOffsetIn = ROBOT_HALF_WIDTH_IN;
        break;
    case TURN_PIVOT_CENTER:
    default:
        break;
    }
}
