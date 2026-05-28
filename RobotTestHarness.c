#include "RobotTestHarness.h"

#include "ES_Configure.h"
#include "ES_Events.h"
#include "RobotDebug.h"
#include "RobotHSM.h"
#include "RobotHardware.h"
#include "RobotIMU.h"
#include "RobotLauncher.h"
#include "RobotMotion.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"
#include "RobotSensors.h"
#include "RobotStepper.h"
#include "IO_Ports.h"
#include "serial.h"
#include "timers.h"

#include <stdio.h>

#if defined(ROBOT_MOTOR_SENSOR_TEST) || defined(ROBOT_KEYBOARD_TEST)
static uint8_t CommandUsesStrafeSpeed(const char *commandName);

static uint8_t CommandUsesStrafeSpeed(const char *commandName)
{
    if (commandName == NULL) {
        return FALSE;
    }
    if ((commandName[0] == 's') && (commandName[1] == 't') &&
            (commandName[2] == 'r')) {
        return TRUE;
    }
    if ((commandName[0] == 'd') && (commandName[1] == 'e') &&
            (commandName[2] == 'm') && (commandName[6] == 's') &&
            (commandName[7] == 't') && (commandName[8] == 'r')) {
        return TRUE;
    }
    return FALSE;
}
#endif

#ifdef ROBOT_MOTOR_SENSOR_TEST
#define BENCH_SENSOR_PERIOD_MS 250u
#define STEPPER_BENCH_BURST_STEPS 20u

typedef enum {
    BENCH_SENSOR_NONE = 0,
    BENCH_SENSOR_TAPE1,
    BENCH_SENSOR_TAPE2,
    BENCH_SENSOR_TAPE3,
    BENCH_SENSOR_TAPE4,
    BENCH_SENSOR_TAPE5,
    BENCH_SENSOR_BUMP_FR,
    BENCH_SENSOR_BUMP_FL,
    BENCH_SENSOR_BUMP_RR,
    BENCH_SENSOR_BUMP_RL,
    BENCH_SENSOR_BEACON,
    BENCH_SENSOR_COUNT,
} BenchSensor_t;

#define BENCH_SENSOR_MASK(sensor) (1u << ((uint8_t) (sensor)))
#define BENCH_SENSOR_ALL_MASK (BENCH_SENSOR_MASK(BENCH_SENSOR_TAPE1) | \
    BENCH_SENSOR_MASK(BENCH_SENSOR_TAPE2) | \
    BENCH_SENSOR_MASK(BENCH_SENSOR_TAPE3) | \
    BENCH_SENSOR_MASK(BENCH_SENSOR_TAPE4) | \
    BENCH_SENSOR_MASK(BENCH_SENSOR_TAPE5) | \
    BENCH_SENSOR_MASK(BENCH_SENSOR_BUMP_FR) | \
    BENCH_SENSOR_MASK(BENCH_SENSOR_BUMP_FL) | \
    BENCH_SENSOR_MASK(BENCH_SENSOR_BUMP_RR) | \
    BENCH_SENSOR_MASK(BENCH_SENSOR_BUMP_RL) | \
    BENCH_SENSOR_MASK(BENCH_SENSOR_BEACON))

static void BenchPrintHelp(void);
static void BenchHandleKey(char key);
static void BenchHandleMotorKey(char key);
static uint8_t BenchHandleSensorKey(char key);
static void BenchToggleSensor(BenchSensor_t sensor, const char *name);
static const char *BenchRunDriveCommand(char key);
static void BenchReapplyDriveCommand(const char *changeName);
static void BenchPrintMotorCommand(const char *commandName);
static void BenchPrintWheelPattern(const char *commandName,
        const char *pattern);
static void BenchRunMotorSequenceStep(void);
static void BenchPrintActiveSensors(void);
static void BenchPrintSensor(BenchSensor_t sensor);
static void BenchPrintTape(TapeSensor_t sensor, const char *name, const char *pinLabel);
static void BenchPrintBump(BumpSensor_t sensor, const char *name, const char *pinLabel);
static void BenchPrintBeacon(void);

static float benchMotorSpeedIPS = MOTOR_BENCH_SPEED_IPS;
static uint8_t benchMotorSequenceStep = 0u;
static uint16_t activeSensorMask = 0u;
static char activeDriveCommandKey = '\0';

void RobotTestHarness_RunMotorSensorBench(void)
{
    unsigned int lastSensorPrintTime;

    TIMERS_Init();
    BenchPrintHelp();
    lastSensorPrintTime = TIMERS_GetTime();

    for (;;) {
        unsigned int now;

        if (!IsReceiveEmpty()) {
            char key = GetChar();
            if ((key != '\r') && (key != '\n') && (key != ' ')) {
                BenchHandleKey(key);
                lastSensorPrintTime = TIMERS_GetTime();
            }
        }

        now = TIMERS_GetTime();
        if ((activeSensorMask != 0u) &&
                ((unsigned int) (now - lastSensorPrintTime) >= BENCH_SENSOR_PERIOD_MS)) {
            BenchPrintActiveSensors();
            lastSensorPrintTime = now;
        }
    }
}

static void BenchPrintHelp(void)
{
    printf("\r\n[BENCH] Direct motor/sensor harness, no HSM\r\n");
    printf("[BENCH] no sensor streaming at startup\r\n");
    printf("[BENCH] ? help, ! hardware pin map, . print active sensors once\r\n");
    printf("[SENSOR] toggle streams: t/y/u/i/o tape 1/2/3/4/5\r\n");
    printf("[SENSOR] toggle streams: f/g/h/j bump FR/FL/RR/RL, b beacon\r\n");
    printf("[SENSOR] p toggle all sensors, l turn all sensor streams off\r\n");
    printf("[MOTOR] 1 forward, 2 reverse, 3 strafe left, 4 strafe right\r\n");
    printf("[MOTOR] 5 turn left center, 6 turn right center\r\n");
    printf("[MOTOR] 7/8 turn left/right about front center\r\n");
    printf("[MOTOR] 9/0 turn left/right about back center\r\n");
    printf("[MOTOR] q/e turn left/right about left center\r\n");
    printf("[MOTOR] a/d turn left/right about right center\r\n");
    printf("[MOTOR] per-wheel: F/G FL fwd/rev, H/J FR fwd/rev\r\n");
    printf("[MOTOR] per-wheel: K/L RL fwd/rev, M/N RR fwd/rev\r\n");
    printf("[MOTOR] strafe right expected wheel signs: FL- FR+ RL+ RR-\r\n");
    printf("[MOTOR] strafe left expected wheel signs:  FL+ FR- RL- RR+\r\n");
    printf("[SHOOTER] v shooter ON, c shooter OFF; x stops drive and shooter\r\n");
    printf("[STEPPER] [ one step forward, ] one step reverse\r\n");
    printf("[STEPPER] { %u steps forward, } %u steps reverse; enable is hard-wired 3.3V\r\n",
            (unsigned int) STEPPER_BENCH_BURST_STEPS,
            (unsigned int) STEPPER_BENCH_BURST_STEPS);
    printf("[MOTOR] strafe commands use fixed %u in/s\r\n",
            (unsigned int) STRAFE_SPEED_IPS);
    printf("[MOTOR] + speed up, - speed down, r reset speed, n next demo command\r\n");
    BenchPrintMotorCommand("current speed");
    printf("\r\n");
}

static void BenchHandleKey(char key)
{
    if (BenchHandleSensorKey(key) == TRUE) {
        return;
    }
    BenchHandleMotorKey(key);
}

static uint8_t BenchHandleSensorKey(char key)
{
    switch (key) {
    case '?':
        BenchPrintHelp();
        return TRUE;
    case '!':
        RobotPlugPlay_PrintConfig();
        return TRUE;
    case '.':
        BenchPrintActiveSensors();
        return TRUE;
    case 'p':
        activeSensorMask = (activeSensorMask == BENCH_SENSOR_ALL_MASK) ? 0u :
                BENCH_SENSOR_ALL_MASK;
        printf("[SENSOR] all streams %s\r\n",
                (activeSensorMask == 0u) ? "OFF" : "ON");
        return TRUE;
    case 'l':
        activeSensorMask = 0u;
        printf("[SENSOR] all streams OFF\r\n");
        return TRUE;
    case 't':
        BenchToggleSensor(BENCH_SENSOR_TAPE1, "tape1");
        return TRUE;
    case 'y':
        BenchToggleSensor(BENCH_SENSOR_TAPE2, "tape2");
        return TRUE;
    case 'u':
        BenchToggleSensor(BENCH_SENSOR_TAPE3, "tape3");
        return TRUE;
    case 'i':
        BenchToggleSensor(BENCH_SENSOR_TAPE4, "tape4");
        return TRUE;
    case 'o':
        BenchToggleSensor(BENCH_SENSOR_TAPE5, "tape5");
        return TRUE;
    case 'f':
        BenchToggleSensor(BENCH_SENSOR_BUMP_FR, "bumpFR");
        return TRUE;
    case 'g':
        BenchToggleSensor(BENCH_SENSOR_BUMP_FL, "bumpFL");
        return TRUE;
    case 'h':
        BenchToggleSensor(BENCH_SENSOR_BUMP_RR, "bumpRR");
        return TRUE;
    case 'j':
        BenchToggleSensor(BENCH_SENSOR_BUMP_RL, "bumpRL");
        return TRUE;
    case 'b':
        BenchToggleSensor(BENCH_SENSOR_BEACON, "beacon");
        return TRUE;
    default:
        return FALSE;
    }
}

static void BenchToggleSensor(BenchSensor_t sensor, const char *name)
{
    uint16_t mask = BENCH_SENSOR_MASK(sensor);

    activeSensorMask ^= mask;
    printf("[SENSOR] %s stream %s\r\n", name,
            (activeSensorMask & mask) ? "ON" : "OFF");
    if (activeSensorMask & mask) {
        BenchPrintSensor(sensor);
    }
}

static void BenchHandleMotorKey(char key)
{
    const char *driveCommandName;

    driveCommandName = BenchRunDriveCommand(key);
    if (driveCommandName != NULL) {
        activeDriveCommandKey = key;
        BenchPrintMotorCommand(driveCommandName);
        return;
    }

    switch (key) {
    case 'x':
        RobotHardware_StopAllOutputs();
        activeDriveCommandKey = '\0';
        printf("[MOTOR] stop\r\n");
        break;
    case 'v':
        RobotLauncher_StartShooter();
        printf("[SHOOTER] ON at duty %u on %s\r\n",
                (unsigned int) SHOOTER_MOTOR_DUTY,
                SHOOTER_MOTOR_PIN_LABEL);
        break;
    case 'c':
        RobotLauncher_StopShooter();
        printf("[SHOOTER] OFF\r\n");
        break;
    case '[':
        RobotStepper_Step(1u, TRUE);
        printf("[STEPPER] 1 step forward on %s\r\n", STEPPER_PIN_LABEL);
        break;
    case ']':
        RobotStepper_Step(1u, FALSE);
        printf("[STEPPER] 1 step reverse on %s\r\n", STEPPER_PIN_LABEL);
        break;
    case '{':
        RobotStepper_Step(STEPPER_BENCH_BURST_STEPS, TRUE);
        printf("[STEPPER] %u steps forward on %s\r\n",
                (unsigned int) STEPPER_BENCH_BURST_STEPS,
                STEPPER_PIN_LABEL);
        break;
    case '}':
        RobotStepper_Step(STEPPER_BENCH_BURST_STEPS, FALSE);
        printf("[STEPPER] %u steps reverse on %s\r\n",
                (unsigned int) STEPPER_BENCH_BURST_STEPS,
                STEPPER_PIN_LABEL);
        break;
    case '+':
    case '=':
        benchMotorSpeedIPS += 1.0f;
        BenchReapplyDriveCommand("speed up");
        break;
    case '-':
    case '_':
        if (benchMotorSpeedIPS > 1.0f) {
            benchMotorSpeedIPS -= 1.0f;
        }
        BenchReapplyDriveCommand("speed down");
        break;
    case 'r':
        benchMotorSpeedIPS = MOTOR_BENCH_SPEED_IPS;
        BenchReapplyDriveCommand("reset speed");
        break;
    case 'n':
        BenchRunMotorSequenceStep();
        break;
    default:
        printf("[BENCH] key '%c' is not bound. Press ? for help.\r\n", key);
        break;
    }
}

static const char *BenchRunDriveCommand(char key)
{
    switch (key) {
    case '1':
        RobotMotion_Forward(benchMotorSpeedIPS);
        return "forward";
    case '2':
        RobotMotion_Reverse(benchMotorSpeedIPS);
        return "reverse";
    case '3':
        RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
        BenchPrintWheelPattern("strafe left", "FL+ FR- RL- RR+");
        return "strafe left";
    case '4':
        RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
        BenchPrintWheelPattern("strafe right", "FL- FR+ RL+ RR-");
        return "strafe right";
    case '5':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, benchMotorSpeedIPS);
        return "turn left about center";
    case '6':
        RobotMotion_TurnRightAbout(TURN_PIVOT_CENTER, benchMotorSpeedIPS);
        return "turn right about center";
    case '7':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_FRONT_CENTER, benchMotorSpeedIPS);
        return "turn left about front center";
    case '8':
        RobotMotion_TurnRightAbout(TURN_PIVOT_FRONT_CENTER, benchMotorSpeedIPS);
        return "turn right about front center";
    case '9':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_BACK_CENTER, benchMotorSpeedIPS);
        return "turn left about back center";
    case '0':
        RobotMotion_TurnRightAbout(TURN_PIVOT_BACK_CENTER, benchMotorSpeedIPS);
        return "turn right about back center";
    case 'q':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_LEFT_CENTER, benchMotorSpeedIPS);
        return "turn left about left center";
    case 'e':
        RobotMotion_TurnRightAbout(TURN_PIVOT_LEFT_CENTER, benchMotorSpeedIPS);
        return "turn right about left center";
    case 'a':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_RIGHT_CENTER, benchMotorSpeedIPS);
        return "turn left about right center";
    case 'd':
        RobotMotion_TurnRightAbout(TURN_PIVOT_RIGHT_CENTER, benchMotorSpeedIPS);
        return "turn right about right center";
    case 'F':
        RobotMotion_TestWheelSpeeds(benchMotorSpeedIPS, 0.0f, 0.0f, 0.0f);
        return "FL only forward";
    case 'G':
        RobotMotion_TestWheelSpeeds(-benchMotorSpeedIPS, 0.0f, 0.0f, 0.0f);
        return "FL only reverse";
    case 'H':
        RobotMotion_TestWheelSpeeds(0.0f, benchMotorSpeedIPS, 0.0f, 0.0f);
        return "FR only forward";
    case 'J':
        RobotMotion_TestWheelSpeeds(0.0f, -benchMotorSpeedIPS, 0.0f, 0.0f);
        return "FR only reverse";
    case 'K':
        RobotMotion_TestWheelSpeeds(0.0f, 0.0f, benchMotorSpeedIPS, 0.0f);
        return "RL only forward";
    case 'L':
        RobotMotion_TestWheelSpeeds(0.0f, 0.0f, -benchMotorSpeedIPS, 0.0f);
        return "RL only reverse";
    case 'M':
        RobotMotion_TestWheelSpeeds(0.0f, 0.0f, 0.0f, benchMotorSpeedIPS);
        return "RR only forward";
    case 'N':
        RobotMotion_TestWheelSpeeds(0.0f, 0.0f, 0.0f, -benchMotorSpeedIPS);
        return "RR only reverse";
    default:
        return NULL;
    }
}

static void BenchReapplyDriveCommand(const char *changeName)
{
    const char *driveCommandName;

    if (activeDriveCommandKey == '\0') {
        BenchPrintMotorCommand(changeName);
        return;
    }

    driveCommandName = BenchRunDriveCommand(activeDriveCommandKey);
    printf("[MOTOR] %s; reapplied %s at %u in/s\r\n",
            changeName,
            driveCommandName,
            (unsigned int) (CommandUsesStrafeSpeed(driveCommandName) ?
            STRAFE_SPEED_IPS : benchMotorSpeedIPS));
}

static void BenchPrintMotorCommand(const char *commandName)
{
    printf("[MOTOR] %s at %u in/s\r\n", commandName,
            (unsigned int) (CommandUsesStrafeSpeed(commandName) ?
            STRAFE_SPEED_IPS : benchMotorSpeedIPS));
}

static void BenchPrintWheelPattern(const char *commandName,
        const char *pattern)
{
    printf("[MOTOR] %s wheel signs: %s\r\n", commandName, pattern);
}

static void BenchRunMotorSequenceStep(void)
{
    switch (benchMotorSequenceStep) {
    case 0u:
        RobotMotion_Forward(benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: forward");
        break;
    case 1u:
        RobotMotion_Reverse(benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: reverse");
        break;
    case 2u:
        RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
        BenchPrintMotorCommand("demo: strafe right");
        break;
    case 3u:
        RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
        BenchPrintMotorCommand("demo: strafe left");
        break;
    case 4u:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: turn left center");
        break;
    case 5u:
        RobotMotion_TurnRightAbout(TURN_PIVOT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: turn right center");
        break;
    case 6u:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_FRONT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: turn left front center");
        break;
    case 7u:
        RobotMotion_TurnRightAbout(TURN_PIVOT_FRONT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: turn right front center");
        break;
    case 8u:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_BACK_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: turn left back center");
        break;
    case 9u:
        RobotMotion_TurnRightAbout(TURN_PIVOT_BACK_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: turn right back center");
        break;
    case 10u:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_LEFT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: turn left left center");
        break;
    case 11u:
        RobotMotion_TurnRightAbout(TURN_PIVOT_LEFT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: turn right left center");
        break;
    case 12u:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_RIGHT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: turn left right center");
        break;
    case 13u:
        RobotMotion_TurnRightAbout(TURN_PIVOT_RIGHT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: turn right right center");
        break;
    default:
        RobotHardware_StopAllOutputs();
        printf("[MOTOR] demo: stop\r\n");
        break;
    }

    benchMotorSequenceStep++;
    if (benchMotorSequenceStep > 14u) {
        benchMotorSequenceStep = 0u;
    }
}

static void BenchPrintActiveSensors(void)
{
    uint8_t sensor;

    if (activeSensorMask == 0u) {
        printf("[SENSOR] no active streams\r\n");
        return;
    }

    for (sensor = 1u; sensor < BENCH_SENSOR_COUNT; sensor++) {
        if (activeSensorMask & BENCH_SENSOR_MASK(sensor)) {
            BenchPrintSensor((BenchSensor_t) sensor);
        }
    }
}

static void BenchPrintSensor(BenchSensor_t sensor)
{
    switch (sensor) {
    case BENCH_SENSOR_TAPE1:
        BenchPrintTape(TAPE_SENSOR_1, "tape1", TAPE_SENSOR_1_PIN_LABEL);
        break;
    case BENCH_SENSOR_TAPE2:
        BenchPrintTape(TAPE_SENSOR_2, "tape2", TAPE_SENSOR_2_PIN_LABEL);
        break;
    case BENCH_SENSOR_TAPE3:
        BenchPrintTape(TAPE_SENSOR_3, "tape3", TAPE_SENSOR_3_PIN_LABEL);
        break;
    case BENCH_SENSOR_TAPE4:
        BenchPrintTape(TAPE_SENSOR_4, "tape4", TAPE_SENSOR_4_PIN_LABEL);
        break;
    case BENCH_SENSOR_TAPE5:
        BenchPrintTape(TAPE_SENSOR_5, "tape5", TAPE_SENSOR_5_PIN_LABEL);
        break;
    case BENCH_SENSOR_BUMP_FR:
        BenchPrintBump(BUMP_SENSOR_1, "bumpFR", BUMP_SENSOR_1_PIN_LABEL);
        break;
    case BENCH_SENSOR_BUMP_FL:
        BenchPrintBump(BUMP_SENSOR_2, "bumpFL", BUMP_SENSOR_2_PIN_LABEL);
        break;
    case BENCH_SENSOR_BUMP_RR:
        BenchPrintBump(BUMP_SENSOR_3, "bumpRR", BUMP_SENSOR_3_PIN_LABEL);
        break;
    case BENCH_SENSOR_BUMP_RL:
        BenchPrintBump(BUMP_SENSOR_4, "bumpRL", BUMP_SENSOR_4_PIN_LABEL);
        break;
    case BENCH_SENSOR_BEACON:
        BenchPrintBeacon();
        break;
    default:
        break;
    }
}

static void BenchPrintTape(TapeSensor_t sensor, const char *name, const char *pinLabel)
{
    printf("[SENSOR] %s raw=%u onTape=%u pin=%s\r\n", name,
            (unsigned int) RobotSensors_ReadTapeDigital(sensor),
            (unsigned int) RobotSensors_IsTapeOn(sensor),
            pinLabel);
}

static void BenchPrintBump(BumpSensor_t sensor, const char *name, const char *pinLabel)
{
    printf("[SENSOR] %s raw=%u bumped=%u pin=%s\r\n", name,
            (unsigned int) RobotSensors_ReadBumpDigital(sensor),
            (unsigned int) RobotSensors_IsBumpOn(sensor),
            pinLabel);
}

static void BenchPrintBeacon(void)
{
    uint16_t rawADC = RobotSensors_ReadBeaconRawADC();
    uint16_t smoothADC = RobotSensors_ReadBeaconADC();

    printf("[SENSOR] beacon rawADC=%u smoothADC=%u distance=%u ft pin=%s\r\n",
            (unsigned int) rawADC,
            (unsigned int) smoothADC,
            (unsigned int) RobotSensors_BeaconDistanceFeetFromADC(smoothADC),
            BEACON_ADC_PIN_LABEL);
}
#endif

#ifdef ROBOT_BEACON_TEST
#define BEACON_BENCH_PERIOD_MS 100u

static void BeaconBenchPrintHelp(void);
static void BeaconBenchPrintReading(void);

void RobotTestHarness_RunBeaconBench(void)
{
    unsigned int lastPrintTime;
    uint8_t streaming = TRUE;

    TIMERS_Init();
    BeaconBenchPrintHelp();
    lastPrintTime = TIMERS_GetTime();

    for (;;) {
        unsigned int now;

        if (!IsReceiveEmpty()) {
            char key = GetChar();
            switch (key) {
            case '?':
                BeaconBenchPrintHelp();
                break;
            case '.':
                BeaconBenchPrintReading();
                break;
            case 'p':
                streaming = TRUE;
                printf("[BEACON] stream ON\r\n");
                break;
            case 'l':
                streaming = FALSE;
                printf("[BEACON] stream paused\r\n");
                break;
            default:
                break;
            }
            lastPrintTime = TIMERS_GetTime();
        }

        now = TIMERS_GetTime();
        if ((streaming != FALSE) &&
                ((unsigned int) (now - lastPrintTime) >= BEACON_BENCH_PERIOD_MS)) {
            BeaconBenchPrintReading();
            lastPrintTime = now;
        }
    }
}

static void BeaconBenchPrintHelp(void)
{
    printf("\r\n[BEACON] Direct beacon detector harness, no HSM\r\n");
    printf("[BEACON] connect peak detector output to %s\r\n", BEACON_ADC_PIN_LABEL);
    printf("[BEACON] smoothing: %u-sample running average\r\n",
            (unsigned int) BEACON_AVERAGE_SAMPLE_COUNT);
    printf("[BEACON] expected ADC: ~%u at 16+ ft, ~%u at 6 ft or closer\r\n",
            (unsigned int) BEACON_ADC_AT_16FT,
            (unsigned int) BEACON_ADC_AT_6FT);
    printf("[BEACON] ? help, . one reading, p stream, l pause\r\n\r\n");
}

static void BeaconBenchPrintReading(void)
{
    uint16_t rawADC = RobotSensors_ReadBeaconRawADC();
    uint16_t smoothADC = RobotSensors_ReadBeaconADC();
    uint8_t distanceFeet = RobotSensors_BeaconDistanceFeetFromADC(smoothADC);

    printf("[BEACON] rawADC=%u smoothADC=%u approxDistance=%u ft pin=%s\r\n",
            (unsigned int) rawADC,
            (unsigned int) smoothADC,
            (unsigned int) distanceFeet,
            BEACON_ADC_PIN_LABEL);
}
#endif

#ifdef ROBOT_GPIO_HIGH_TEST
#define GPIO_PINS_VW (PIN3 | PIN4 | PIN5 | PIN6 | PIN7 | PIN8)
#define GPIO_PINS_XY (PIN3 | PIN4 | PIN5 | PIN6 | PIN7 | PIN8 | PIN9 | PIN10 | PIN11 | PIN12)
#ifdef GPIO_HIGH_INCLUDE_SERIAL
#define GPIO_PINS_Z GPIO_PINS_XY
#else
#define GPIO_PINS_Z (PIN3 | PIN4 | PIN5 | PIN6 | PIN7 | PIN8 | PIN9 | PIN11)
#endif

static void GPIOHighDriveAll(void);
static void GPIOHighPrintHelp(void);
static void GPIOHighPrintPort(const char *name, uint8_t port, uint16_t pins);

void RobotTestHarness_RunGPIOHighBench(void)
{
    GPIOHighDriveAll();
    GPIOHighPrintHelp();

    for (;;) {
        if (!IsReceiveEmpty()) {
            char key = GetChar();
            switch (key) {
            case '?':
                GPIOHighPrintHelp();
                break;
            case 'h':
                GPIOHighDriveAll();
                printf("[GPIO] all selected pins re-driven HIGH\r\n");
                break;
            default:
                break;
            }
        }
    }
}

static void GPIOHighDriveAll(void)
{
    IO_PortsSetPortOutputs(PORTV, GPIO_PINS_VW);
    IO_PortsSetPortOutputs(PORTW, GPIO_PINS_VW);
    IO_PortsSetPortOutputs(PORTX, GPIO_PINS_XY);
    IO_PortsSetPortOutputs(PORTY, GPIO_PINS_XY);
    IO_PortsSetPortOutputs(PORTZ, GPIO_PINS_Z);

    IO_PortsSetPortBits(PORTV, GPIO_PINS_VW);
    IO_PortsSetPortBits(PORTW, GPIO_PINS_VW);
    IO_PortsSetPortBits(PORTX, GPIO_PINS_XY);
    IO_PortsSetPortBits(PORTY, GPIO_PINS_XY);
    IO_PortsSetPortBits(PORTZ, GPIO_PINS_Z);
}

static void GPIOHighPrintHelp(void)
{
    printf("\r\n[GPIO] GPIO-high bench harness, no HSM\r\n");
    printf("[GPIO] Every listed pin is configured as digital output HIGH.\r\n");
    printf("[GPIO] ? help, h re-drive all listed pins HIGH\r\n");
#ifndef GPIO_HIGH_INCLUDE_SERIAL
    printf("[GPIO] Z10/Z12 skipped to keep serial terminal alive.\r\n");
    printf("[GPIO] Build with HW_EXTRA_DEFINES=GPIO_HIGH_INCLUDE_SERIAL to include them.\r\n");
#endif
    GPIOHighPrintPort("V", PORTV, GPIO_PINS_VW);
    GPIOHighPrintPort("W", PORTW, GPIO_PINS_VW);
    GPIOHighPrintPort("X", PORTX, GPIO_PINS_XY);
    GPIOHighPrintPort("Y", PORTY, GPIO_PINS_XY);
    GPIOHighPrintPort("Z", PORTZ, GPIO_PINS_Z);
    printf("\r\n");
}

static void GPIOHighPrintPort(const char *name, uint8_t port, uint16_t pins)
{
    printf("[GPIO] Port %s pins mask=0x%04X readback=0x%04X\r\n",
            name,
            (unsigned int) pins,
            (unsigned int) (IO_PortsReadPort(port) & pins));
}
#endif

#ifdef ROBOT_KEYBOARD_TEST
static uint8_t PostEventByType(ES_EventTyp_t eventType, uint16_t eventParam,
        const char *sourceLabel);
static uint8_t HandleMotorTestKey(char key);
static void PrintMotorTestHelp(void);
static void PrintMotorCommand(const char *commandName);
static void RunMotorSequenceStep(void);
static uint8_t GetEventForKey(char key, ES_EventTyp_t *eventType);
static char GetKeyForEventIndex(uint8_t eventIndex);
static uint16_t GetDefaultParamForEvent(ES_EventTyp_t eventType);
static void PrintBindingForKey(char key);

static uint8_t queryNextKey = FALSE;
static uint8_t motorTestMode = FALSE;
static float motorTestSpeedIPS = MOTOR_SPEED_IPS;
static uint8_t motorSequenceStep = 0u;

uint8_t RobotTestHarness_CheckKeyboard(void)
{
    char key;
    ES_EventTyp_t eventType;

    if (IsReceiveEmpty()) {
        return FALSE;
    }

    key = GetChar();

    if ((key == '\r') || (key == '\n') || (key == ' ')) {
        return TRUE;
    }

    if (queryNextKey == TRUE) {
        queryNextKey = FALSE;
        PrintBindingForKey(key);
        return TRUE;
    }

    if (key == '*') {
        motorTestMode = motorTestMode ? FALSE : TRUE;
        RobotMotion_Stop();
        printf("[MOTOR] direct motor test %s\r\n", motorTestMode ? "ON" : "OFF");
        if (motorTestMode == TRUE) {
            PrintMotorTestHelp();
        }
        return TRUE;
    }

    if ((motorTestMode == TRUE) && (key == '?')) {
        PrintMotorTestHelp();
        return TRUE;
    }

    if (key == '?') {
        queryNextKey = TRUE;
        printf("[TEST] type a key to inspect its bound event\r\n");
        return TRUE;
    }
    if (key == '!') {
        RobotPlugPlay_PrintConfig();
        return TRUE;
    }
    if (key == '.') {
        RobotDebug_PrintCurrentState();
        return TRUE;
    }
    if (key == '#') {
        RobotDebug_PrintModuleVariables();
        return TRUE;
    }
    if (key == '$') {
        RobotIMU_PrintDebugSnapshot();
        return TRUE;
    }
    if (key == '@') {
        RobotIMU_ToggleDebugStream();
        return TRUE;
    }

    if (motorTestMode == TRUE) {
        return HandleMotorTestKey(key);
    }

    if (GetEventForKey(key, &eventType) == TRUE) {
        return PostEventByType(eventType, GetDefaultParamForEvent(eventType),
                EventNames[eventType]);
    }

    printf("[TEST] key '%c' is not bound. Press ? for bindings.\r\n", key);
    return TRUE;
}

void RobotTestHarness_PrintHelp(void)
{
    printf("\r\n[TEST] Keyboard HSM test harness\r\n");
    printf("[TEST] key posts the event bound to that key\r\n");
    printf("[TEST] ? then key prints the event bound to that key\r\n");
    printf("[TEST] ! prints enabled hardware and pin hookups\r\n");
    printf("[TEST] . prints the current active state path\r\n");
    printf("[TEST] # prints module variables\r\n");
    printf("[TEST] $ prints one BNO055 raw/status snapshot\r\n");
    printf("[TEST] @ toggles live BNO055 raw/status stream\r\n");
    printf("[TEST] * toggles direct motor-control test mode\r\n");
    printf("[TEST] key ranges: 0-9, a-z, A-Z in EventNames[] order\r\n");
    printf("\r\n");
}

static uint8_t HandleMotorTestKey(char key)
{
    switch (key) {
    case 'w':
        RobotMotion_Forward(motorTestSpeedIPS);
        PrintMotorCommand("forward");
        break;
    case 's':
        RobotMotion_Reverse(motorTestSpeedIPS);
        PrintMotorCommand("reverse");
        break;
    case 'a':
        RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
        PrintMotorCommand("strafe left");
        break;
    case 'd':
        RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
        PrintMotorCommand("strafe right");
        break;
    case 'q':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("turn left about center");
        break;
    case 'e':
        RobotMotion_TurnRightAbout(TURN_PIVOT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("turn right about center");
        break;
    case '1':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_FRONT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("turn left about front center");
        break;
    case '2':
        RobotMotion_TurnRightAbout(TURN_PIVOT_FRONT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("turn right about front center");
        break;
    case '3':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_BACK_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("turn left about back center");
        break;
    case '4':
        RobotMotion_TurnRightAbout(TURN_PIVOT_BACK_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("turn right about back center");
        break;
    case '5':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_LEFT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("turn left about left center");
        break;
    case '6':
        RobotMotion_TurnRightAbout(TURN_PIVOT_LEFT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("turn right about left center");
        break;
    case '7':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_RIGHT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("turn left about right center");
        break;
    case '8':
        RobotMotion_TurnRightAbout(TURN_PIVOT_RIGHT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("turn right about right center");
        break;
    case 'x':
        RobotMotion_Stop();
        printf("[MOTOR] stop\r\n");
        break;
    case '+':
    case '=':
        motorTestSpeedIPS += 1.0f;
        PrintMotorCommand("speed up");
        break;
    case '-':
    case '_':
        if (motorTestSpeedIPS > 1.0f) {
            motorTestSpeedIPS -= 1.0f;
        }
        PrintMotorCommand("speed down");
        break;
    case 'r':
        motorTestSpeedIPS = MOTOR_SPEED_IPS;
        PrintMotorCommand("reset speed");
        break;
    case 'n':
        RunMotorSequenceStep();
        break;
    default:
        printf("[MOTOR] key '%c' is not a motor command. Press ? for motor help.\r\n", key);
        break;
    }

    return TRUE;
}

static void PrintMotorTestHelp(void)
{
    printf("\r\n[MOTOR] Direct motor-control mode\r\n");
    printf("[MOTOR] * exit motor mode, ? print this map, x stop\r\n");
    printf("[MOTOR] w forward, s reverse, a strafe left, d strafe right\r\n");
    printf("[MOTOR] q turn left about center, e turn right about center\r\n");
    printf("[MOTOR] 1/2 turn left/right about front center\r\n");
    printf("[MOTOR] 3/4 turn left/right about back center\r\n");
    printf("[MOTOR] 5/6 turn left/right about left center\r\n");
    printf("[MOTOR] 7/8 turn left/right about right center\r\n");
    printf("[MOTOR] strafe commands use fixed %u in/s\r\n",
            (unsigned int) STRAFE_SPEED_IPS);
    printf("[MOTOR] + speed up, - speed down, r reset speed, n next demo command\r\n");
    PrintMotorCommand("current speed");
    printf("\r\n");
}

static void PrintMotorCommand(const char *commandName)
{
    printf("[MOTOR] %s at %u in/s\r\n", commandName,
            (unsigned int) (CommandUsesStrafeSpeed(commandName) ?
            STRAFE_SPEED_IPS : motorTestSpeedIPS));
}

static void RunMotorSequenceStep(void)
{
    switch (motorSequenceStep) {
    case 0u:
        RobotMotion_Forward(motorTestSpeedIPS);
        PrintMotorCommand("demo: forward");
        break;
    case 1u:
        RobotMotion_Reverse(motorTestSpeedIPS);
        PrintMotorCommand("demo: reverse");
        break;
    case 2u:
        RobotMotion_StrafeRight(STRAFE_SPEED_IPS);
        PrintMotorCommand("demo: strafe right");
        break;
    case 3u:
        RobotMotion_StrafeLeft(STRAFE_SPEED_IPS);
        PrintMotorCommand("demo: strafe left");
        break;
    case 4u:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("demo: turn left center");
        break;
    case 5u:
        RobotMotion_TurnRightAbout(TURN_PIVOT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("demo: turn right center");
        break;
    case 6u:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_FRONT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("demo: turn left front center");
        break;
    case 7u:
        RobotMotion_TurnRightAbout(TURN_PIVOT_FRONT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("demo: turn right front center");
        break;
    case 8u:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_BACK_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("demo: turn left back center");
        break;
    case 9u:
        RobotMotion_TurnRightAbout(TURN_PIVOT_BACK_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("demo: turn right back center");
        break;
    case 10u:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_LEFT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("demo: turn left left center");
        break;
    case 11u:
        RobotMotion_TurnRightAbout(TURN_PIVOT_LEFT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("demo: turn right left center");
        break;
    case 12u:
        RobotMotion_TurnLeftAbout(TURN_PIVOT_RIGHT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("demo: turn left right center");
        break;
    case 13u:
        RobotMotion_TurnRightAbout(TURN_PIVOT_RIGHT_CENTER, motorTestSpeedIPS);
        PrintMotorCommand("demo: turn right right center");
        break;
    default:
        RobotMotion_Stop();
        printf("[MOTOR] demo: stop\r\n");
        break;
    }

    motorSequenceStep++;
    if (motorSequenceStep > 14u) {
        motorSequenceStep = 0u;
    }
}

static uint8_t PostEventByType(ES_EventTyp_t eventType, uint16_t eventParam,
        const char *sourceLabel)
{
    ES_Event event;

    event.EventType = eventType;
    event.EventParam = eventParam;

    printf("[EVENT] post %s (%u)\r\n", sourceLabel, (unsigned int) eventParam);
    return PostRobotHSM(event);
}

static void PrintBindingForKey(char key)
{
    ES_EventTyp_t eventType;

    if (GetEventForKey(key, &eventType) == TRUE) {
        printf("[TEST] %c -> %s (%u)\r\n", key, EventNames[eventType],
                (unsigned int) GetDefaultParamForEvent(eventType));
    } else {
        printf("[TEST] key '%c' is not bound\r\n", key);
    }
}

static uint8_t GetEventForKey(char key, ES_EventTyp_t *eventType)
{
    uint8_t i;

    for (i = 0u; i < NUMBEROFEVENTS; i++) {
        if (GetKeyForEventIndex(i) == key) {
            *eventType = (ES_EventTyp_t) i;
            return TRUE;
        }
    }

    return FALSE;
}

static char GetKeyForEventIndex(uint8_t eventIndex)
{
    if (eventIndex < 10u) {
        return (char) ('0' + eventIndex);
    }

    eventIndex -= 10u;
    if (eventIndex < 26u) {
        return (char) ('a' + eventIndex);
    }

    eventIndex -= 26u;
    return (char) ('A' + eventIndex);
}

static uint16_t GetDefaultParamForEvent(ES_EventTyp_t eventType)
{
    switch (eventType) {
    case ES_TIMEOUT:
        return FIND_FRONT_IMU_TIMER;
    case BeaconADCIncreaseEvent:
        return 100u;
    case MaxSignalFoundEvent:
        return 512u;
    case TapeSensor1OnEvent:
    case TapeSensor1OffEvent:
        return 1u;
    case TapeSensor2OnEvent:
    case TapeSensor2OffEvent:
        return 2u;
    case TapeSensor3OnEvent:
    case TapeSensor3OffEvent:
        return 3u;
    case TapeSensor4OnEvent:
    case TapeSensor4OffEvent:
        return 4u;
    case TapeSensor5OnEvent:
    case TapeSensor5OffEvent:
    case TapeSensor5LowToHighEvent:
        return 5u;
    case Solenoid1OnEvent:
        return 1u;
    case Solenoid2OnEvent:
        return 2u;
    case Solenoid3OnEvent:
        return 3u;
    case Solenoid4OnEvent:
        return 4u;
    case Solenoid5OnEvent:
        return 5u;
    case Solenoid6OnEvent:
        return 6u;
    case BumpSensor1OnEvent:
    case BumpSensor1OffEvent:
        return 1u;
    case BumpSensor2OnEvent:
    case BumpSensor2OffEvent:
        return 2u;
    case BumpSensor3OnEvent:
    case BumpSensor3OffEvent:
        return 3u;
    case BumpSensor4OnEvent:
    case BumpSensor4OffEvent:
        return 4u;
    case RealignedEvent:
    case PositionRealignedEvent:
        return ALIGN_REALIGNED_SOURCE_MANUAL;
    default:
        return 0u;
    }
}
#else
uint8_t RobotTestHarness_CheckKeyboard(void)
{
    return FALSE;
}

void RobotTestHarness_PrintHelp(void)
{
}
#endif

#if !defined(ROBOT_MOTOR_SENSOR_TEST)
void RobotTestHarness_RunMotorSensorBench(void)
{
}
#endif

#if !defined(ROBOT_BEACON_TEST)
void RobotTestHarness_RunBeaconBench(void)
{
}
#endif

#if !defined(ROBOT_GPIO_HIGH_TEST)
void RobotTestHarness_RunGPIOHighBench(void)
{
}
#endif
