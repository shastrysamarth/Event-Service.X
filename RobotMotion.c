#include "RobotMotion.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
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
static uint8_t distanceMovePaused = FALSE;
static DistanceAxis_t distanceAxis = DISTANCE_AXIS_Y;
static int8_t distanceDirection = 1;
/* DEPRECATED for open-loop timing: was IMU odometry target; kept for debug getters. */
static float distanceTargetInches = 0.0f;
/* Open-loop distance: elapsed ms vs (inches / MOTOR_SPEED_IPS) * 1000 */
static uint32_t distanceStartMs = 0u;
static uint32_t distanceDurationMs = 0u;
static uint32_t distanceRemainingMs = 0u;
static const char *lastMotionCommand = "Init";
static const char *lastMotionPivot = "none";
/* Last command/pivot we already logged, so we only print on a real change.
 * RobotMotion stores command/pivot as stable string literals, so comparing
 * pointers is enough to detect a transition. */
static const char *prevLoggedCommand = "Init";
static const char *prevLoggedPivot = "none";
/* Caller name passed to the most recent RobotMotion_Stop(); consumed (and
 * cleared) by LogMotionChange so the "[MOTOR] ... (stop by <fn>)" line shows
 * exactly which function commanded the stop. */
static const char *stopCaller = (const char *) 0;
static float lastFrontLeftIPS = 0.0f;
static float lastFrontRightIPS = 0.0f;
static float lastRearLeftIPS = 0.0f;
static float lastRearRightIPS = 0.0f;
static int lastFrontLeftDuty = 0;
static int lastFrontRightDuty = 0;
static int lastRearLeftDuty = 0;
static int lastRearRightDuty = 0;

static void ConfigureDirectionPins(void);
static uint8_t EnsurePWMReady(void);
/* The final four args tune this command only: FL, FR, RL, RR. Direct wheel
 * tests bypass this so they still output the exact speed requested.
 */
static void SetChassisVelocity(float vxIPS, float vyIPS, float omegaRadPerSec,
        float frontLeftScale, float frontRightScale,
        float rearLeftScale, float rearRightScale);
static void SetDriveWheels(float frontLeft, float frontRight, float rearLeft, float rearRight);
static void BrakeAllWheels(void);
static void SetMotor(DriveMotor_t motor, int speedDuty);
static void SetMotorDirection(DriveMotor_t motor, int forward);
static void SetMotorBrake(DriveMotor_t motor);
static int SpeedToDuty(float wheelSpeedIPS);
static float AbsFloat(float value);
static void PrintFixedInline(float value, const char *unit);
static const char *PivotName(TurnPivot_t pivot);
static void PivotOffset_LeftTurn(TurnPivot_t pivot, float *xOffsetIn, float *yOffsetIn);
static void PivotOffset_RightTurn(TurnPivot_t pivot, float *xOffsetIn, float *yOffsetIn);
static void LogMotionChange(void);

/* Emits a "[MOTOR] control change" line whenever the drive command or pivot
 * actually changes. Called from every command setter so the log is captured
 * synchronously at the moment the command is issued, regardless of which
 * service/state issued it (the keyboard checker that used to host this lives
 * last in EVENT_CHECK_LIST and gets starved while sensors keep posting). */
static void LogMotionChange(void)
{
    const char *caller = stopCaller;

    /* Consume the pending stop attribution every call so a deduped stop can't
     * leak its caller onto a later, unrelated command transition. */
    stopCaller = (const char *) 0;

    if ((lastMotionCommand == prevLoggedCommand) &&
            (lastMotionPivot == prevLoggedPivot)) {
        return;
    }
#if (defined(DEBUG) || defined(ROBOT_DEBUG)) && ROBOT_LOG_MOTOR
    if (caller != (const char *) 0) {
        printf("[MOTOR] control change: %s/%s -> %s/%s (stop by %s)\r\n",
                prevLoggedCommand, prevLoggedPivot,
                lastMotionCommand, lastMotionPivot, caller);
    } else {
        printf("[MOTOR] control change: %s/%s -> %s/%s\r\n",
                prevLoggedCommand, prevLoggedPivot,
                lastMotionCommand, lastMotionPivot);
    }
#else
    (void) caller;
#endif
    prevLoggedCommand = lastMotionCommand;
    prevLoggedPivot = lastMotionPivot;
}

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

void RobotMotion_StopImpl(const char *caller)
{
    stopCaller = (caller != (const char *) 0) ? caller : "?";
    lastMotionCommand = "stop";
    lastMotionPivot = "none";
    LogMotionChange();
    /* Active (dynamic) braking instead of coasting: see BrakeAllWheels. */
    BrakeAllWheels();
}

void RobotMotion_Forward(float speedIPS)
{
    lastMotionCommand = "forward";
    lastMotionPivot = "none";
    LogMotionChange();
    SetChassisVelocity(0.0f, speedIPS, 0.0f,
            1.08f, 1.00f, 1.08f, 1.00f);
}

void RobotMotion_Reverse(float speedIPS)
{
    lastMotionCommand = "reverse";
    lastMotionPivot = "none";
    LogMotionChange();
    SetChassisVelocity(0.0f, -speedIPS, 0.0f,
            1.08f, 1.00f, 1.08f, 1.00f);
}

void RobotMotion_StrafeRight(float speedIPS)
{
    lastMotionCommand = "strafe-right";
    lastMotionPivot = "none";
    LogMotionChange();
    SetChassisVelocity(-speedIPS, 0.0f, 0.0f,
            1.00f, 1.00f, 1.00f, 1.07f);
}

void RobotMotion_StrafeLeft(float speedIPS)
{
    lastMotionCommand = "strafe-left";
    lastMotionPivot = "none";
    LogMotionChange();
    SetChassisVelocity(speedIPS, 0.0f, 0.0f,
            1.15f, 1.00f, 1.00f, 1.00f);
}

void RobotMotion_TestWheelSpeeds(float frontLeftIPS, float frontRightIPS,
        float rearLeftIPS, float rearRightIPS)
{
    lastMotionCommand = "manual-wheel-test";
    lastMotionPivot = "none";
    LogMotionChange();
    SetDriveWheels(frontLeftIPS, frontRightIPS, rearLeftIPS, rearRightIPS);
}

void RobotMotion_TurnLeftAbout(TurnPivot_t pivot, float speedIPS)
{
    float xOffset;
    float yOffset;
    float omega;

    PivotOffset_LeftTurn(pivot, &xOffset, &yOffset);
    omega = speedIPS / (ROBOT_HALF_WIDTH_IN + ROBOT_HALF_LENGTH_IN);
    lastMotionCommand = "turn-left";
    lastMotionPivot = PivotName(pivot);
    LogMotionChange();
    SetChassisVelocity(omega * yOffset, -omega * xOffset, omega,
            1.00f, 1.00f, 1.00f, 1.00f);
}

void RobotMotion_TurnRightAbout(TurnPivot_t pivot, float speedIPS)
{
    float xOffset;
    float yOffset;
    float omega;

    PivotOffset_RightTurn(pivot, &xOffset, &yOffset);
    omega = -speedIPS / (ROBOT_HALF_WIDTH_IN + ROBOT_HALF_LENGTH_IN);
    lastMotionCommand = "turn-right";
    lastMotionPivot = PivotName(pivot);
    LogMotionChange();
    SetChassisVelocity(omega * yOffset, -omega * xOffset, omega,
            1.00f, 1.00f, 1.00f, 1.00f);
}

void RobotMotion_StartDistanceMove(DistanceAxis_t axis, int8_t direction, float targetInches)
{
    RobotMotion_StartDistanceMoveAtSpeed(axis, direction, targetInches, MOTOR_SPEED_IPS);
}

void RobotMotion_StartDistanceMoveAtSpeed(DistanceAxis_t axis, int8_t direction,
        float targetInches, float speedIPS)
{
    float durationFloat;

    distanceMoveActive = TRUE;
    distanceMovePaused = FALSE;
    distanceAxis = axis;
    distanceDirection = (direction >= 0) ? 1 : -1;
    distanceTargetInches = targetInches;

    if (speedIPS < 0.0f) {
        speedIPS = -speedIPS;
    }
    if (speedIPS < 0.01f) {
        speedIPS = MOTOR_SPEED_IPS;
    }

    durationFloat = (targetInches / speedIPS) * 1000.0f;
    if (durationFloat < 1.0f) {
        durationFloat = 1.0f;
    }
    distanceDurationMs = (uint32_t) (durationFloat + 0.5f);
    distanceRemainingMs = distanceDurationMs;
    distanceStartMs = ES_Timer_GetTime();

#if ROBOT_REALTIME_TRACE
    printf("[MOTOR] distance-move start axis=%s dir=%d target=",
            (axis == DISTANCE_AXIS_X) ? "X" : "Y", (int) direction);
    PrintFixedInline(targetInches, "in");
    printf(" speed=");
    PrintFixedInline(speedIPS, "ips");
    printf(" dur=%lums\r\n", (unsigned long) distanceDurationMs);
#endif
}

void RobotMotion_StopDistanceMove(void)
{
    distanceMoveActive = FALSE;
    distanceMovePaused = FALSE;
    distanceRemainingMs = 0u;
#if ROBOT_REALTIME_TRACE
    printf("[MOTOR] distance-move stop\r\n");
#endif
}

void RobotMotion_PauseDistanceMove(void)
{
    uint32_t now;
    uint32_t elapsed;

    if ((distanceMoveActive == FALSE) || (distanceMovePaused == TRUE)) {
        return;
    }

    now = ES_Timer_GetTime();
    elapsed = now - distanceStartMs;
    if (elapsed >= distanceDurationMs) {
        distanceRemainingMs = 1u;
    } else {
        distanceRemainingMs = distanceDurationMs - elapsed;
    }
    distanceMoveActive = FALSE;
    distanceMovePaused = TRUE;
#if ROBOT_REALTIME_TRACE
    printf("[MOTOR] distance-move pause remainingMs=%lu\r\n",
            (unsigned long) distanceRemainingMs);
#endif
}

void RobotMotion_ResumeDistanceMove(void)
{
    if (distanceMovePaused == FALSE) {
        return;
    }

    distanceDurationMs = (distanceRemainingMs == 0u) ? 1u : distanceRemainingMs;
    distanceStartMs = ES_Timer_GetTime();
    distanceMoveActive = TRUE;
    distanceMovePaused = FALSE;
#if ROBOT_REALTIME_TRACE
    printf("[MOTOR] distance-move resume remainingMs=%lu\r\n",
            (unsigned long) distanceDurationMs);
#endif
}

uint8_t RobotMotion_IsDistanceMoveActive(void)
{
    return (distanceMoveActive || distanceMovePaused) ? TRUE : FALSE;
}

uint8_t RobotMotion_IsDistanceMoveComplete(void)
{
    uint32_t now;
    uint32_t elapsed;

    if ((distanceMoveActive == FALSE) || (distanceMovePaused == TRUE)) {
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

const char *RobotMotion_GetCommandName(void)
{
    return lastMotionCommand;
}

const char *RobotMotion_GetPivotName(void)
{
    return lastMotionPivot;
}

void RobotMotion_DebugPrintCurrentCommand(const char *context)
{
#if ROBOT_REALTIME_TRACE
    printf("[MOTOR] %s control=%s pivot=%s ", context,
            lastMotionCommand,
            lastMotionPivot);
    printf("FL=");
    PrintFixedInline(lastFrontLeftIPS, "ips");
    printf("/duty=%d ", lastFrontLeftDuty);
    printf("FR=");
    PrintFixedInline(lastFrontRightIPS, "ips");
    printf("/duty=%d ", lastFrontRightDuty);
    printf("RL=");
    PrintFixedInline(lastRearLeftIPS, "ips");
    printf("/duty=%d ", lastRearLeftDuty);
    printf("RR=");
    PrintFixedInline(lastRearRightIPS, "ips");
    printf("/duty=%d\r\n", lastRearRightDuty);
#else
    (void) context;
#endif
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

static void SetChassisVelocity(float vxIPS, float vyIPS, float omegaRadPerSec,
        float frontLeftScale, float frontRightScale,
        float rearLeftScale, float rearRightScale)
{
    float radiusSum = ROBOT_HALF_WIDTH_IN + ROBOT_HALF_LENGTH_IN;

    /* Mecanum X-drive mix, with +vx = strafe right and +vy = forward.
     * Pure strafe right should be FL/RR reverse and FR/RL forward.
     * Pure strafe left should be FL/RR forward and FR/RL reverse. */
    float frontLeft = vyIPS - vxIPS - (omegaRadPerSec * radiusSum);
    float frontRight = vyIPS + vxIPS + (omegaRadPerSec * radiusSum);
    float rearLeft = vyIPS + vxIPS - (omegaRadPerSec * radiusSum);
    float rearRight = vyIPS - vxIPS + (omegaRadPerSec * radiusSum);

    frontLeft *= frontLeftScale;
    frontRight *= frontRightScale;
    rearLeft *= rearLeftScale;
    rearRight *= rearRightScale;

    SetDriveWheels(frontLeft, frontRight, rearLeft, rearRight);
}

static void SetDriveWheels(float frontLeft, float frontRight, float rearLeft, float rearRight)
{
    lastFrontLeftIPS = frontLeft;
    lastFrontRightIPS = frontRight;
    lastRearLeftIPS = rearLeft;
    lastRearRightIPS = rearRight;
    lastFrontLeftDuty = SpeedToDuty(frontLeft) * MOTOR_FL_POLARITY;
    lastFrontRightDuty = SpeedToDuty(frontRight) * MOTOR_FR_POLARITY;
    lastRearLeftDuty = SpeedToDuty(rearLeft) * MOTOR_RL_POLARITY;
    lastRearRightDuty = SpeedToDuty(rearRight) * MOTOR_RR_POLARITY;

    SetMotor(MOTOR_FRONT_LEFT, lastFrontLeftDuty);
    SetMotor(MOTOR_FRONT_RIGHT, lastFrontRightDuty);
    SetMotor(MOTOR_REAR_LEFT, lastRearLeftDuty);
    SetMotor(MOTOR_REAR_RIGHT, lastRearRightDuty);

#if ROBOT_REALTIME_TRACE
    printf("[MOTOR] %s pivot=%s FL=%d FR=%d RL=%d RR=%d\r\n",
            lastMotionCommand, lastMotionPivot,
            lastFrontLeftDuty, lastFrontRightDuty,
            lastRearLeftDuty, lastRearRightDuty);
#endif
}

static void BrakeAllWheels(void)
{
    lastFrontLeftIPS = 0.0f;
    lastFrontRightIPS = 0.0f;
    lastRearLeftIPS = 0.0f;
    lastRearRightIPS = 0.0f;
    lastFrontLeftDuty = 0;
    lastFrontRightDuty = 0;
    lastRearLeftDuty = 0;
    lastRearRightDuty = 0;

    SetMotorBrake(MOTOR_FRONT_LEFT);
    SetMotorBrake(MOTOR_FRONT_RIGHT);
    SetMotorBrake(MOTOR_REAR_LEFT);
    SetMotorBrake(MOTOR_REAR_RIGHT);

#if ROBOT_REALTIME_TRACE
    printf("[MOTOR] brake (dynamic) FL/FR/RL/RR\r\n");
#endif
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

/* L298N dynamic braking (datasheet truth table): with ENA asserted (PWM high)
 * and IN1 == IN2, the bridge shorts the motor leads so back-EMF actively brakes
 * the wheel. Coasting (the old stop) instead drove ENA low, disconnecting the
 * motor. We tie both direction pins low and drive the enable PWM to MAX_PWM. */
static void SetMotorBrake(DriveMotor_t motor)
{
#if ROBOT_PLUGPLAY_USE_DRIVE_MOTORS
    switch (motor) {
    case MOTOR_FRONT_LEFT:
        MOTOR_FL_IN1_LAT = 0;
        MOTOR_FL_IN2_LAT = 0;
        PWM_SetDutyCycle(MOTOR_FL_PWM_PIN, MAX_PWM);
        break;
    case MOTOR_FRONT_RIGHT:
        MOTOR_FR_IN1_LAT = 0;
        MOTOR_FR_IN2_LAT = 0;
        PWM_SetDutyCycle(MOTOR_FR_PWM_PIN, MAX_PWM);
        break;
    case MOTOR_REAR_LEFT:
        MOTOR_RL_IN1_LAT = 0;
        MOTOR_RL_IN2_LAT = 0;
        PWM_SetDutyCycle(MOTOR_RL_PWM_PIN, MAX_PWM);
        break;
    case MOTOR_REAR_RIGHT:
        MOTOR_RR_IN1_LAT = 0;
        MOTOR_RR_IN2_LAT = 0;
        PWM_SetDutyCycle(MOTOR_RR_PWM_PIN, MAX_PWM);
        break;
    default:
        break;
    }
#else
    (void) motor;
#endif
}

static int SpeedToDuty(float wheelSpeedIPS)
{
    float magnitude = AbsFloat(wheelSpeedIPS);
    int duty = (int) (magnitude * MOTOR_DUTY_PER_IPS);

    if ((duty > 0) && (duty < (int) MOTOR_MIN_ACTIVE_DUTY)) {
        duty = (int) MOTOR_MIN_ACTIVE_DUTY;
    }
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

static void PrintFixedInline(float value, const char *unit)
{
    int32_t scaled = (int32_t) (value * 100.0f);
    uint32_t magnitude;

    if (scaled < 0) {
        magnitude = (uint32_t) (-scaled);
        printf("-%lu.%02lu%s",
                (unsigned long) (magnitude / 100u),
                (unsigned long) (magnitude % 100u),
                unit);
    } else {
        magnitude = (uint32_t) scaled;
        printf("%lu.%02lu%s",
                (unsigned long) (magnitude / 100u),
                (unsigned long) (magnitude % 100u),
                unit);
    }
}

static const char *PivotName(TurnPivot_t pivot)
{
    switch (pivot) {
    case TURN_PIVOT_FRONT_CENTER:
        return "front-center";
    case TURN_PIVOT_BACK_CENTER:
        return "back-center";
    case TURN_PIVOT_LEFT_CENTER:
        return "left-center";
    case TURN_PIVOT_RIGHT_CENTER:
        return "right-center";
    case TURN_PIVOT_CENTER:
        return "center";
    default:
        return "unknown";
    }
}

static void PivotOffset_LeftTurn(TurnPivot_t pivot, float *xOffsetIn, float *yOffsetIn)
{
    *xOffsetIn = 0.0f;
    *yOffsetIn = 0.0f;

    switch (pivot) {
    case TURN_PIVOT_FRONT_CENTER:
        *yOffsetIn = -4.2f;
        break;
    case TURN_PIVOT_BACK_CENTER:
        *yOffsetIn = 4.25f;
        *xOffsetIn = 0.05f;
        break;
    case TURN_PIVOT_LEFT_CENTER:
        *xOffsetIn = -1.8f;
        break;
    case TURN_PIVOT_RIGHT_CENTER:
        *xOffsetIn = 1.87f;
        break;
    case TURN_PIVOT_CENTER:
        *xOffsetIn = -0.10f;
        *yOffsetIn = -0.25f;
        break;
    default:
        break;
    }
}

static void PivotOffset_RightTurn(TurnPivot_t pivot, float *xOffsetIn, float *yOffsetIn)
{
    *xOffsetIn = 0.0f;
    *yOffsetIn = 0.0f;

    switch (pivot) {
    case TURN_PIVOT_FRONT_CENTER:
        *yOffsetIn = -4.3f;
        *xOffsetIn = 0.7f;
        break;
    case TURN_PIVOT_BACK_CENTER:
        *yOffsetIn = 4.25f;
        *xOffsetIn = 0.1f;
        break;
    case TURN_PIVOT_LEFT_CENTER:
        *xOffsetIn = -1.8f;
        break;
    case TURN_PIVOT_RIGHT_CENTER:
        *xOffsetIn = 1.87f;
        break;
    case TURN_PIVOT_CENTER:
        *xOffsetIn = 0.8f;
        *yOffsetIn = 0.05f;
        break;
    default:
        break;
    }
}
