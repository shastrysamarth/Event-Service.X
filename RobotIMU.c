#include "RobotIMU.h"

#include <xc.h>
#include <stdio.h>

#include "BOARD.h"
#include "ES_Timers.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"

#define BNO055_CHIP_ID_REG 0x00u
#define BNO055_PAGE_ID_REG 0x07u
#define BNO055_ACCEL_DATA_X_LSB_REG 0x08u
#define BNO055_GYRO_DATA_X_LSB_REG 0x14u
#define BNO055_EULER_H_LSB_REG 0x1Au
#define BNO055_QUATERNION_DATA_W_LSB_REG 0x20u
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_REG 0x28u
#define BNO055_GRAVITY_DATA_X_LSB_REG 0x2Eu
#define BNO055_TEMP_REG 0x34u
#define BNO055_UNIT_SEL_REG 0x3Bu
#define BNO055_OPR_MODE_REG 0x3Du
#define BNO055_PWR_MODE_REG 0x3Eu
#define BNO055_SYS_TRIGGER_REG 0x3Fu
#define BNO055_CALIB_STAT_REG 0x35u
#define BNO055_ST_RESULT_REG 0x36u
#define BNO055_SYS_STATUS_REG 0x39u
#define BNO055_SYS_ERR_REG 0x3Au

#define BNO055_ADDR_LOW 0x28u
#define BNO055_ADDR_HIGH 0x29u
#define BNO055_CHIP_ID 0xA0u
#define BNO055_OPERATION_MODE_CONFIG 0x00u
#define BNO055_OPERATION_MODE_NDOF 0x0Cu
#define BNO055_POWER_MODE_NORMAL 0x00u
#define BNO055_ACCEL_LSB_PER_MPS2 100.0f
#define MPS2_TO_INPS2 39.3701f
#define I2C_TIMEOUT_LOOPS 40000u

#define BNO_SDA_TRIS PORTW06_TRIS
#define BNO_SDA_LAT PORTW06_LAT
#define BNO_SDA_BIT PORTW06_BIT
#define BNO_SCL_TRIS PORTW08_TRIS
#define BNO_SCL_LAT PORTW08_LAT
#define BNO_SCL_BIT PORTW08_BIT

static uint8_t imuReady = FALSE;
static uint8_t bnoAddress = BNO055_ADDR_LOW;
static float headingOffsetDeg = 0.0f;
static float headingDeg = 0.0f;
static float xInches = 0.0f;
static float yInches = 0.0f;
static float xVelocityIPS = 0.0f;
static float yVelocityIPS = 0.0f;
static float xAccelIPS2 = 0.0f;
static float yAccelIPS2 = 0.0f;
static float zGyroDPS = 0.0f;
static uint32_t lastUpdateMs = 0;
static uint16_t stationaryMs = 0u;
static uint8_t isStationary = TRUE;
static uint8_t debugStreamEnabled = FALSE;
static uint32_t lastDebugStreamMs = 0u;
static int16_t rawEuler[3] = {0, 0, 0};
static int16_t rawGyro[3] = {0, 0, 0};
static int16_t rawAccel[3] = {0, 0, 0};
static int16_t rawLinearAccel[3] = {0, 0, 0};
static int16_t rawGravity[3] = {0, 0, 0};
static int16_t rawQuaternion[4] = {0, 0, 0, 0};
static uint8_t rawCalib = 0u;
static uint8_t rawTempC = 0u;
static uint8_t rawSelfTest = 0u;
static uint8_t rawSystemStatus = 0u;
static uint8_t rawSystemError = 0u;
static uint8_t rawOperationMode = 0u;
static uint8_t lastReadOk = FALSE;
static uint8_t lastModeWasNDOF = FALSE;

static void I2CInit(void);
static uint8_t I2CStart(void);
static uint8_t I2CRestart(void);
static uint8_t I2CStop(void);
static uint8_t I2CSend(uint8_t data);
static uint8_t I2CReceive(uint8_t *data, uint8_t ack);
static uint8_t I2CWaitIdle(void);
static void DelayMs(uint32_t ms);
static void I2CDelay(void);
static uint8_t I2CWaitSclHigh(void);
static uint8_t BNO055ProbeAddress(uint8_t address);
static uint8_t BNO055Find(void);
static uint8_t BNO055Write8(uint8_t reg, uint8_t value);
static uint8_t BNO055ReadLen(uint8_t reg, uint8_t *buffer, uint8_t length);
static uint8_t BNO055Read8(uint8_t reg, uint8_t *value);
static int16_t ReadS16(const uint8_t *buffer);
static void StoreVector3(const uint8_t *buffer, int16_t vector[3]);
static void StoreQuaternion(const uint8_t *buffer, int16_t quaternion[4]);
static int16_t ApplyDeadbandS16(int16_t value, int16_t deadband);
static uint16_t AbsS16(int16_t value);
static float NormalizeHeading(float heading);
static float SmallestHeadingError(float target, float current);
static void PrintFixedQ4(int16_t value);
static void PrintFixed100(int16_t value);
static void PrintFixed14(int16_t value);
static void PrintFloat100(float value);

uint8_t RobotIMU_Init(void)
{
#if ROBOT_PLUGPLAY_USE_BNO055
    I2CInit();
    DelayMs(50u);

    if (BNO055Find() != TRUE) {
        imuReady = FALSE;
#ifdef ROBOT_DEBUG
        printf("[DEBUG][IMU] BNO055 not found on SDA W6 / SCL W8\r\n");
#endif
        return FALSE;
    }

    imuReady = RobotIMU_BeginNDOF();
    RobotIMU_ZeroAll();
#ifdef ROBOT_DEBUG
    printf("[DEBUG][IMU] BNO055 %s at 0x%02X on SDA W6 / SCL W8\r\n",
            imuReady ? "ready" : "init failed", bnoAddress);
#endif
    return imuReady;
#else
    imuReady = FALSE;
    RobotIMU_ZeroAll();
    return TRUE;
#endif
}

uint8_t RobotIMU_BeginNDOF(void)
{
#if ROBOT_PLUGPLAY_USE_BNO055
    uint8_t chipId = 0;
    uint16_t tries;

    if (BNO055Find() != TRUE) {
        return FALSE;
    }

    if (BNO055Write8(BNO055_OPR_MODE_REG, BNO055_OPERATION_MODE_CONFIG) != TRUE) {
        return FALSE;
    }
    DelayMs(30u);
    BNO055Write8(BNO055_SYS_TRIGGER_REG, 0x20u);
    DelayMs(700u);

    for (tries = 0u; tries < 100u; tries++) {
        if ((BNO055Read8(BNO055_CHIP_ID_REG, &chipId) == TRUE) &&
                (chipId == BNO055_CHIP_ID)) {
            break;
        }
        DelayMs(10u);
    }
    if (tries == 100u) {
        return FALSE;
    }

    if (BNO055Write8(BNO055_OPR_MODE_REG, BNO055_OPERATION_MODE_CONFIG) != TRUE) {
        return FALSE;
    }
    DelayMs(30u);
    BNO055Write8(BNO055_PAGE_ID_REG, 0x00u);
    BNO055Write8(BNO055_PWR_MODE_REG, BNO055_POWER_MODE_NORMAL);
    DelayMs(10u);
    BNO055Write8(BNO055_SYS_TRIGGER_REG, 0x00u);
    BNO055Write8(BNO055_UNIT_SEL_REG, 0x00u);
    DelayMs(10u);
    if (BNO055Write8(BNO055_OPR_MODE_REG, BNO055_OPERATION_MODE_NDOF) != TRUE) {
        return FALSE;
    }
    DelayMs(20u);
    return TRUE;
#else
    return TRUE;
#endif
}

uint8_t RobotIMU_IsFullyCalibrated(void)
{
#if ROBOT_PLUGPLAY_USE_BNO055
    uint8_t cal = 0;

    if (BNO055Read8(BNO055_CALIB_STAT_REG, &cal) != TRUE) {
        return FALSE;
    }

    return (((cal >> 6) & 0x03u) == 3u &&
            ((cal >> 4) & 0x03u) == 3u &&
            ((cal >> 2) & 0x03u) == 3u &&
            (cal & 0x03u) == 3u) ? TRUE : FALSE;
#else
    return TRUE;
#endif
}

void RobotIMU_Update(void)
{
#if ROBOT_PLUGPLAY_USE_BNO055
    uint8_t buffer[6];
    uint8_t gyroBuffer[6];
    uint8_t tempBuffer[8];
    uint32_t now = ES_Timer_GetTime();
    uint32_t elapsedMs = now - lastUpdateMs;
    float dt;
    int16_t axRaw;
    int16_t ayRaw;
    int16_t azRaw;
    int16_t gxRaw;
    int16_t gyRaw;
    int16_t gzRaw;
    uint8_t planarAccelIsStill;
    uint8_t gyroIsStill;

    if (imuReady == FALSE) {
        lastUpdateMs = now;
        return;
    }

    if (elapsedMs == 0u) {
        return;
    }

    lastReadOk = TRUE;

    if (BNO055ReadLen(BNO055_EULER_H_LSB_REG, buffer, 6u) == TRUE) {
        StoreVector3(buffer, rawEuler);
        headingDeg = NormalizeHeading(((float) ReadS16(buffer)) / 16.0f - headingOffsetDeg);
    } else {
        lastReadOk = FALSE;
    }

    if (BNO055ReadLen(BNO055_LINEAR_ACCEL_DATA_X_LSB_REG, buffer, 6u) == TRUE) {
        dt = ((float) elapsedMs) / 1000.0f;
        StoreVector3(buffer, rawLinearAccel);
        axRaw = ReadS16(&buffer[0]);
        ayRaw = ReadS16(&buffer[2]);
        azRaw = ReadS16(&buffer[4]);

        int16_t axRawStill = axRaw;
        int16_t ayRawStill = ayRaw;

        if (BNO055ReadLen(BNO055_GYRO_DATA_X_LSB_REG, gyroBuffer, 6u) == TRUE) {
            StoreVector3(gyroBuffer, rawGyro);
            gxRaw = ReadS16(&gyroBuffer[0]);
            gyRaw = ReadS16(&gyroBuffer[2]);
            gzRaw = ReadS16(&gyroBuffer[4]);
        } else {
            lastReadOk = FALSE;
            gxRaw = rawGyro[0];
            gyRaw = rawGyro[1];
            gzRaw = rawGyro[2];
        }

        if (BNO055ReadLen(BNO055_ACCEL_DATA_X_LSB_REG, tempBuffer, 6u) == TRUE) {
            StoreVector3(tempBuffer, rawAccel);
        } else {
            lastReadOk = FALSE;
        }
        if (BNO055ReadLen(BNO055_GRAVITY_DATA_X_LSB_REG, tempBuffer, 6u) == TRUE) {
            StoreVector3(tempBuffer, rawGravity);
        } else {
            lastReadOk = FALSE;
        }
        if (BNO055ReadLen(BNO055_QUATERNION_DATA_W_LSB_REG, tempBuffer, 8u) == TRUE) {
            StoreQuaternion(tempBuffer, rawQuaternion);
        } else {
            lastReadOk = FALSE;
        }
        if (BNO055Read8(BNO055_CALIB_STAT_REG, &rawCalib) != TRUE) {
            lastReadOk = FALSE;
        }
        if (BNO055Read8(BNO055_TEMP_REG, &rawTempC) != TRUE) {
            lastReadOk = FALSE;
        }
        if (BNO055Read8(BNO055_ST_RESULT_REG, &rawSelfTest) != TRUE) {
            lastReadOk = FALSE;
        }
        if (BNO055Read8(BNO055_SYS_STATUS_REG, &rawSystemStatus) != TRUE) {
            lastReadOk = FALSE;
        }
        if (BNO055Read8(BNO055_SYS_ERR_REG, &rawSystemError) != TRUE) {
            lastReadOk = FALSE;
        }
        if (BNO055Read8(BNO055_OPR_MODE_REG, &rawOperationMode) != TRUE) {
            lastReadOk = FALSE;
        }

        if (rawOperationMode != BNO055_OPERATION_MODE_NDOF) {
            lastModeWasNDOF = FALSE;
            RobotIMU_EnsureNDOF();
            lastUpdateMs = now;
            return;
        }
        lastModeWasNDOF = TRUE;

        planarAccelIsStill = ((AbsS16(axRawStill) <= IMU_STILL_ACCEL_RAW) &&
                (AbsS16(ayRawStill) <= IMU_STILL_ACCEL_RAW)) ? TRUE : FALSE;
        gyroIsStill = (AbsS16(gzRaw) <= IMU_STILL_GYRO_RAW) ? TRUE : FALSE;
        (void) azRaw;
        (void) gxRaw;
        (void) gyRaw;

        axRaw = ApplyDeadbandS16(axRaw, IMU_ACCEL_INTEGRATE_DEADBAND_RAW);
        ayRaw = ApplyDeadbandS16(ayRaw, IMU_ACCEL_INTEGRATE_DEADBAND_RAW);

        /*
         * DEPRECATED: field-frame rotation via cosf/sinf removed to avoid
         * soft-float libm stack overflow on PIC32MX (1 KB stack).
         * Position integration is no longer used for control; distance moves
         * are timer-based and alignment is heading-only.
         * xAccelIPS2/yAccelIPS2 are kept as raw body-frame values for debug.
         */
        xAccelIPS2 = (((float) axRaw) / BNO055_ACCEL_LSB_PER_MPS2) * MPS2_TO_INPS2;
        yAccelIPS2 = (((float) ayRaw) / BNO055_ACCEL_LSB_PER_MPS2) * MPS2_TO_INPS2;
        zGyroDPS = ((float) gzRaw) / 16.0f;

        if ((planarAccelIsStill == TRUE) && (gyroIsStill == TRUE)) {
            if (stationaryMs < IMU_STATIONARY_CONFIRM_MS) {
                stationaryMs += (elapsedMs > 0xFFFFu) ? 0xFFFFu : (uint16_t) elapsedMs;
            }
            if (stationaryMs >= IMU_STATIONARY_CONFIRM_MS) {
                xVelocityIPS = 0.0f;
                yVelocityIPS = 0.0f;
                xAccelIPS2 = 0.0f;
                yAccelIPS2 = 0.0f;
                isStationary = TRUE;
            }
        } else {
            stationaryMs = 0u;
            isStationary = FALSE;
        }

        if (isStationary == FALSE) {
            xVelocityIPS += xAccelIPS2 * dt;
            yVelocityIPS += yAccelIPS2 * dt;
        }
        xInches += xVelocityIPS * dt;
        yInches += yVelocityIPS * dt;
    } else {
        lastReadOk = FALSE;
    }

    lastUpdateMs = now;
#endif
}

void RobotIMU_ZeroAll(void)
{
#if ROBOT_PLUGPLAY_USE_BNO055
    uint8_t buffer[2];

    if (BNO055ReadLen(BNO055_EULER_H_LSB_REG, buffer, 2u) == TRUE) {
        headingOffsetDeg = ((float) ReadS16(buffer)) / 16.0f;
    }
#else
    headingOffsetDeg = 0.0f;
#endif
    headingDeg = 0.0f;
    RobotIMU_ZeroPositionVelocity();
    lastUpdateMs = ES_Timer_GetTime();
}

void RobotIMU_ZeroHeading(void)
{
#if ROBOT_PLUGPLAY_USE_BNO055
    uint8_t buffer[2];

    if (BNO055ReadLen(BNO055_EULER_H_LSB_REG, buffer, 2u) == TRUE) {
        headingOffsetDeg = ((float) ReadS16(buffer)) / 16.0f;
    }
#else
    headingOffsetDeg = 0.0f;
#endif
    headingDeg = 0.0f;
}

void RobotIMU_ZeroPositionVelocity(void)
{
    xInches = 0.0f;
    yInches = 0.0f;
    xVelocityIPS = 0.0f;
    yVelocityIPS = 0.0f;
    xAccelIPS2 = 0.0f;
    yAccelIPS2 = 0.0f;
    stationaryMs = 0u;
    isStationary = TRUE;
}

float RobotIMU_GetHeadingDeg(void)
{
    return headingDeg;
}

float RobotIMU_GetXInches(void)
{
    return xInches;
}

float RobotIMU_GetYInches(void)
{
    return yInches;
}

float RobotIMU_GetHeadingErrorToZeroDeg(void)
{
    return SmallestHeadingError(0.0f, headingDeg);
}

uint8_t RobotIMU_IsReady(void)
{
    return imuReady;
}

float RobotIMU_GetHeadingOffsetDeg(void)
{
    return headingOffsetDeg;
}

float RobotIMU_GetXVelocityIPS(void)
{
    return xVelocityIPS;
}

float RobotIMU_GetYVelocityIPS(void)
{
    return yVelocityIPS;
}

uint8_t RobotIMU_IsStationary(void)
{
    return isStationary;
}

float RobotIMU_GetXAccelIPS2(void)
{
    return xAccelIPS2;
}

float RobotIMU_GetYAccelIPS2(void)
{
    return yAccelIPS2;
}

float RobotIMU_GetZGyroDPS(void)
{
    return zGyroDPS;
}

void RobotIMU_PrintDebugSnapshot(void)
{
#if ROBOT_PLUGPLAY_USE_BNO055
    printf("\r\n[IMU] BNO055 @0x%02X ready=%u readOk=%u mode=0x%02X%s selftest=0x%02X sys=%u err=%u temp=%uC\r\n",
            bnoAddress,
            (unsigned int) imuReady,
            (unsigned int) lastReadOk,
            (unsigned int) rawOperationMode,
            (rawOperationMode == BNO055_OPERATION_MODE_NDOF) ? " NDOF" : "",
            (unsigned int) rawSelfTest,
            (unsigned int) rawSystemStatus,
            (unsigned int) rawSystemError,
            (unsigned int) rawTempC);
    printf("[IMU] cal S/G/A/M=%u/%u/%u/%u stationary=%u stationaryMs=%u\r\n",
            (unsigned int) ((rawCalib >> 6) & 0x03u),
            (unsigned int) ((rawCalib >> 4) & 0x03u),
            (unsigned int) ((rawCalib >> 2) & 0x03u),
            (unsigned int) (rawCalib & 0x03u),
            (unsigned int) isStationary,
            (unsigned int) stationaryMs);
    printf("[IMU] modeGuard=%s\r\n", lastModeWasNDOF ? "NDOF" : "RECOVERING");

    printf("[IMU] eul H/R/P=");
    PrintFixedQ4(rawEuler[0]);
    printf("/");
    PrintFixedQ4(rawEuler[1]);
    printf("/");
    PrintFixedQ4(rawEuler[2]);
    printf(" corrH=");
    PrintFixedQ4((int16_t) (headingDeg * 16.0f));

    printf(" gyro=");
    PrintFixedQ4(rawGyro[0]);
    printf("/");
    PrintFixedQ4(rawGyro[1]);
    printf("/");
    PrintFixedQ4(rawGyro[2]);
    printf("\r\n");

    printf("[IMU] lin=");
    PrintFixed100(rawLinearAccel[0]);
    printf("/");
    PrintFixed100(rawLinearAccel[1]);
    printf("/");
    PrintFixed100(rawLinearAccel[2]);
    printf(" acc=");
    PrintFixed100(rawAccel[0]);
    printf("/");
    PrintFixed100(rawAccel[1]);
    printf("/");
    PrintFixed100(rawAccel[2]);
    printf(" grav=");
    PrintFixed100(rawGravity[0]);
    printf("/");
    PrintFixed100(rawGravity[1]);
    printf("/");
    PrintFixed100(rawGravity[2]);
    printf("\r\n");

    printf("[IMU] quat=");
    PrintFixed14(rawQuaternion[0]);
    printf("/");
    PrintFixed14(rawQuaternion[1]);
    printf("/");
    PrintFixed14(rawQuaternion[2]);
    printf("/");
    PrintFixed14(rawQuaternion[3]);
    printf(" pos x/y=");
    PrintFloat100(xInches);
    printf("/");
    PrintFloat100(yInches);
    printf(" in vel x/y=");
    PrintFloat100(xVelocityIPS);
    printf("/");
    PrintFloat100(yVelocityIPS);
    printf(" in/s\r\n");
#else
    printf("\r\n[IMU] BNO055 disabled\r\n");
#endif
}

void RobotIMU_SetDebugStream(uint8_t enabled)
{
    debugStreamEnabled = enabled ? TRUE : FALSE;
    lastDebugStreamMs = 0u;
}

void RobotIMU_ToggleDebugStream(void)
{
    RobotIMU_SetDebugStream(debugStreamEnabled ? FALSE : TRUE);
    printf("[IMU] live debug %s\r\n", debugStreamEnabled ? "ON" : "OFF");
}

uint8_t RobotIMU_IsDebugStreamEnabled(void)
{
    return debugStreamEnabled;
}

uint8_t RobotIMU_EnsureNDOF(void)
{
#if ROBOT_PLUGPLAY_USE_BNO055
    uint8_t mode = 0u;

    if (imuReady == FALSE) {
        return FALSE;
    }

    if (BNO055Read8(BNO055_OPR_MODE_REG, &mode) != TRUE) {
        lastReadOk = FALSE;
        return FALSE;
    }

    rawOperationMode = mode;
    if (mode == BNO055_OPERATION_MODE_NDOF) {
        lastModeWasNDOF = TRUE;
        return TRUE;
    }

    if (BNO055Write8(BNO055_OPR_MODE_REG, BNO055_OPERATION_MODE_NDOF) != TRUE) {
        lastReadOk = FALSE;
        return FALSE;
    }
    DelayMs(20u);

    if (BNO055Read8(BNO055_OPR_MODE_REG, &mode) != TRUE) {
        lastReadOk = FALSE;
        return FALSE;
    }

    rawOperationMode = mode;
    lastModeWasNDOF = (mode == BNO055_OPERATION_MODE_NDOF) ? TRUE : FALSE;
#ifdef ROBOT_DEBUG
    printf("[DEBUG][IMU] recovered BNO055 mode 0x%02X%s\r\n",
            (unsigned int) mode,
            lastModeWasNDOF ? " NDOF" : "");
#endif
    if (lastModeWasNDOF == TRUE) {
        RobotIMU_ZeroPositionVelocity();
        lastUpdateMs = ES_Timer_GetTime();
    }
    return lastModeWasNDOF;
#else
    return TRUE;
#endif
}

void RobotIMU_DebugStreamTick(void)
{
    uint32_t now;

    if (debugStreamEnabled == FALSE) {
        return;
    }

    now = ES_Timer_GetTime();
    if ((lastDebugStreamMs == 0u) ||
            ((now - lastDebugStreamMs) >= IMU_DEBUG_STREAM_PERIOD_MS)) {
        lastDebugStreamMs = now;
        RobotIMU_PrintDebugSnapshot();
    }
}

static void I2CInit(void)
{
    uint8_t pulseCount;

    (void) BNO055_I2C_BAUD_HZ;

    AD1PCFGSET = (1u << 12) | (1u << 14);
    BNO_SDA_LAT = 0;
    BNO_SCL_LAT = 0;
    BNO_SDA_TRIS = 1;
    BNO_SCL_TRIS = 1;
    I2CDelay();

    for (pulseCount = 0u; pulseCount < 9u && BNO_SDA_BIT == 0; pulseCount++) {
        BNO_SCL_TRIS = 0;
        I2CDelay();
        BNO_SCL_TRIS = 1;
        I2CWaitSclHigh();
        I2CDelay();
    }
    I2CStop();
}

static uint8_t I2CStart(void)
{
    BNO_SDA_TRIS = 1;
    BNO_SCL_TRIS = 1;
    if (I2CWaitSclHigh() == FALSE) {
        return FALSE;
    }
    I2CDelay();
    if (BNO_SDA_BIT == 0) {
        return FALSE;
    }
    BNO_SDA_TRIS = 0;
    I2CDelay();
    BNO_SCL_TRIS = 0;
    I2CDelay();
    return TRUE;
}

static uint8_t I2CRestart(void)
{
    BNO_SDA_TRIS = 1;
    BNO_SCL_TRIS = 1;
    if (I2CWaitSclHigh() == FALSE) {
        return FALSE;
    }
    I2CDelay();
    BNO_SDA_TRIS = 0;
    I2CDelay();
    BNO_SCL_TRIS = 0;
    I2CDelay();
    return TRUE;
}

static uint8_t I2CStop(void)
{
    BNO_SDA_TRIS = 0;
    I2CDelay();
    BNO_SCL_TRIS = 1;
    if (I2CWaitSclHigh() == FALSE) {
        return FALSE;
    }
    I2CDelay();
    BNO_SDA_TRIS = 1;
    I2CDelay();
    return TRUE;
}

static uint8_t I2CSend(uint8_t data)
{
    uint8_t bitIndex;
    uint8_t acked;

    for (bitIndex = 0u; bitIndex < 8u; bitIndex++) {
        if ((data & 0x80u) != 0u) {
            BNO_SDA_TRIS = 1;
        } else {
            BNO_SDA_TRIS = 0;
        }
        I2CDelay();
        BNO_SCL_TRIS = 1;
        if (I2CWaitSclHigh() == FALSE) {
            return FALSE;
        }
        I2CDelay();
        BNO_SCL_TRIS = 0;
        I2CDelay();
        data <<= 1;
    }

    BNO_SDA_TRIS = 1;
    I2CDelay();
    BNO_SCL_TRIS = 1;
    if (I2CWaitSclHigh() == FALSE) {
        return FALSE;
    }
    I2CDelay();
    acked = (BNO_SDA_BIT == 0) ? TRUE : FALSE;
    BNO_SCL_TRIS = 0;
    I2CDelay();
    return acked;
}

static uint8_t I2CReceive(uint8_t *data, uint8_t ack)
{
    uint8_t bitIndex;
    uint8_t value = 0u;

    BNO_SDA_TRIS = 1;
    for (bitIndex = 0u; bitIndex < 8u; bitIndex++) {
        value <<= 1;
        BNO_SCL_TRIS = 1;
        if (I2CWaitSclHigh() == FALSE) {
            return FALSE;
        }
        I2CDelay();
        if (BNO_SDA_BIT != 0) {
            value |= 1u;
        }
        BNO_SCL_TRIS = 0;
        I2CDelay();
    }
    *data = value;

    BNO_SDA_TRIS = ack ? 0 : 1;
    I2CDelay();
    BNO_SCL_TRIS = 1;
    if (I2CWaitSclHigh() == FALSE) {
        return FALSE;
    }
    I2CDelay();
    BNO_SCL_TRIS = 0;
    BNO_SDA_TRIS = 1;
    I2CDelay();
    return TRUE;
}

static uint8_t I2CWaitIdle(void)
{
    return TRUE;
}

static void DelayMs(uint32_t ms)
{
    uint32_t start;
    uint32_t ticks;

    while (ms > 0u) {
        start = _CP0_GET_COUNT();
        ticks = BOARD_GetPBClock() / 2000u;
        while ((_CP0_GET_COUNT() - start) < ticks) {
        }
        ms--;
    }
}

static void I2CDelay(void)
{
    uint32_t count;

    for (count = 0u; count < 220u; count++) {
        asm volatile("nop");
    }
}

static uint8_t I2CWaitSclHigh(void)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;

    while (BNO_SCL_BIT == 0) {
        if (timeout-- == 0u) {
            return FALSE;
        }
    }
    return TRUE;
}

static uint8_t BNO055ProbeAddress(uint8_t address)
{
    uint8_t savedAddress = bnoAddress;
    uint8_t chipId = 0;

    bnoAddress = address;
    if ((BNO055Read8(BNO055_CHIP_ID_REG, &chipId) == TRUE) &&
            (chipId == BNO055_CHIP_ID)) {
        return TRUE;
    }

    bnoAddress = savedAddress;
    return FALSE;
}

static uint8_t BNO055Find(void)
{
    if (BNO055ProbeAddress(BNO055_ADDR_LOW) == TRUE) {
        return TRUE;
    }
    if (BNO055ProbeAddress(BNO055_ADDR_HIGH) == TRUE) {
        return TRUE;
    }
    return FALSE;
}

static uint8_t BNO055Write8(uint8_t reg, uint8_t value)
{
    uint8_t ok = TRUE;

    ok &= I2CStart();
    ok &= I2CSend((uint8_t) (bnoAddress << 1));
    ok &= I2CSend(reg);
    ok &= I2CSend(value);
    ok &= I2CStop();
    return ok;
}

static uint8_t BNO055ReadLen(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t i;
    uint8_t ok = TRUE;

    ok &= I2CStart();
    ok &= I2CSend((uint8_t) (bnoAddress << 1));
    ok &= I2CSend(reg);
    ok &= I2CRestart();
    ok &= I2CSend((uint8_t) ((bnoAddress << 1) | 1u));

    for (i = 0; i < length; i++) {
        ok &= I2CReceive(&buffer[i], (i + 1u) < length);
    }

    ok &= I2CStop();
    return ok;
}

static uint8_t BNO055Read8(uint8_t reg, uint8_t *value)
{
    return BNO055ReadLen(reg, value, 1u);
}

static int16_t ReadS16(const uint8_t *buffer)
{
    return (int16_t) ((((uint16_t) buffer[1]) << 8) | buffer[0]);
}

static void StoreVector3(const uint8_t *buffer, int16_t vector[3])
{
    vector[0] = ReadS16(&buffer[0]);
    vector[1] = ReadS16(&buffer[2]);
    vector[2] = ReadS16(&buffer[4]);
}

static void StoreQuaternion(const uint8_t *buffer, int16_t quaternion[4])
{
    quaternion[0] = ReadS16(&buffer[0]);
    quaternion[1] = ReadS16(&buffer[2]);
    quaternion[2] = ReadS16(&buffer[4]);
    quaternion[3] = ReadS16(&buffer[6]);
}

static int16_t ApplyDeadbandS16(int16_t value, int16_t deadband)
{
    return (AbsS16(value) <= (uint16_t) deadband) ? 0 : value;
}

static uint16_t AbsS16(int16_t value)
{
    return (value < 0) ? (uint16_t) (-value) : (uint16_t) value;
}

static float NormalizeHeading(float heading)
{
    while (heading >= 180.0f) {
        heading -= 360.0f;
    }
    while (heading < -180.0f) {
        heading += 360.0f;
    }
    return heading;
}

static float SmallestHeadingError(float target, float current)
{
    return NormalizeHeading(target - current);
}

static void PrintFixedQ4(int16_t value)
{
    uint16_t magnitude;
    uint16_t whole;
    uint16_t frac;

    if (value < 0) {
        printf("-");
        magnitude = (uint16_t) (-value);
    } else {
        magnitude = (uint16_t) value;
    }

    whole = magnitude / 16u;
    frac = magnitude % 16u;
    printf("%u.%02u", whole, (uint16_t) ((uint32_t) frac * 100u / 16u));
}

static void PrintFixed100(int16_t value)
{
    uint16_t magnitude;
    uint16_t whole;
    uint16_t frac;

    if (value < 0) {
        printf("-");
        magnitude = (uint16_t) (-value);
    } else {
        magnitude = (uint16_t) value;
    }

    whole = magnitude / 100u;
    frac = magnitude % 100u;
    printf("%u.%02u", whole, frac);
}

static void PrintFixed14(int16_t value)
{
    uint16_t magnitude;
    uint16_t whole;
    uint16_t frac;

    if (value < 0) {
        printf("-");
        magnitude = (uint16_t) (-value);
    } else {
        magnitude = (uint16_t) value;
    }

    whole = magnitude / 16384u;
    frac = magnitude % 16384u;
    printf("%u.%04u", whole, (uint16_t) ((uint32_t) frac * 10000u / 16384u));
}

static void PrintFloat100(float value)
{
    int32_t scaled = (int32_t) (value * 100.0f);
    uint32_t magnitude;

    if (scaled < 0) {
        printf("-");
        magnitude = (uint32_t) (-scaled);
    } else {
        magnitude = (uint32_t) scaled;
    }

    printf("%lu.%02lu",
            (unsigned long) (magnitude / 100u),
            (unsigned long) (magnitude % 100u));
}
