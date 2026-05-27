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
#include "IO_Ports.h"
#include "serial.h"
#include "timers.h"

#include <stdio.h>

#ifdef ROBOT_MOTOR_SENSOR_TEST
#define BENCH_SENSOR_PERIOD_MS 250u

typedef enum {
    BENCH_SENSOR_NONE,
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
    BENCH_SENSOR_ALL,
} BenchSensor_t;

static void BenchPrintHelp(void);
static void BenchHandleKey(char key);
static void BenchHandleMotorKey(char key);
static uint8_t BenchHandleSensorKey(char key);
static void BenchPrintMotorCommand(const char *commandName);
static void BenchRunMotorSequenceStep(void);
static void BenchPrintSelectedSensor(void);
static void BenchPrintTape(TapeSensor_t sensor, const char *name, const char *pinLabel);
static void BenchPrintBump(BumpSensor_t sensor, const char *name, const char *pinLabel);
static void BenchPrintBeacon(void);

static float benchMotorSpeedIPS = MOTOR_SPEED_IPS;
static uint8_t benchMotorSequenceStep = 0u;
static BenchSensor_t selectedSensor = BENCH_SENSOR_NONE;

void RobotTestHarness_RunMotorSensorBench(void)
{
    unsigned int lastSensorPrintTime;

    TIMERS_Init();
    BenchPrintHelp();
    selectedSensor = BENCH_SENSOR_ALL;
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
        if ((selectedSensor != BENCH_SENSOR_NONE) &&
                ((unsigned int) (now - lastSensorPrintTime) >= BENCH_SENSOR_PERIOD_MS)) {
            BenchPrintSelectedSensor();
            lastSensorPrintTime = now;
        }
    }
}

static void BenchPrintHelp(void)
{
    printf("\r\n[BENCH] Direct motor/sensor harness, no HSM\r\n");
    printf("[BENCH] ? help, ! hardware pin map, . print selected sensor once\r\n");
    printf("[BENCH] p stream all sensors, l pause sensor stream\r\n");
    printf("[BENCH] t/y/u/i/o stream tape sensors 1/2/3/4/5\r\n");
    printf("[BENCH] f/g/h/j stream bump FR/FL/RR/RL\r\n");
    printf("[BENCH] b stream beacon ADC and distance bucket\r\n");
    printf("[MOTOR] w forward, s reverse, a strafe left, d strafe right, x stop\r\n");
    printf("[MOTOR] q/e turn left/right about center\r\n");
    printf("[MOTOR] 1/2 front pivot, 3/4 back pivot, 5/6 left pivot, 7/8 right pivot\r\n");
    printf("[SHOOTER] v shooter ON, c shooter OFF; x stops drive and shooter\r\n");
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
        BenchPrintSelectedSensor();
        return TRUE;
    case 'p':
        selectedSensor = BENCH_SENSOR_ALL;
        BenchPrintSelectedSensor();
        return TRUE;
    case 'l':
        selectedSensor = BENCH_SENSOR_NONE;
        printf("[SENSOR] stream paused\r\n");
        return TRUE;
    case 't':
        selectedSensor = BENCH_SENSOR_TAPE1;
        BenchPrintSelectedSensor();
        return TRUE;
    case 'y':
        selectedSensor = BENCH_SENSOR_TAPE2;
        BenchPrintSelectedSensor();
        return TRUE;
    case 'u':
        selectedSensor = BENCH_SENSOR_TAPE3;
        BenchPrintSelectedSensor();
        return TRUE;
    case 'i':
        selectedSensor = BENCH_SENSOR_TAPE4;
        BenchPrintSelectedSensor();
        return TRUE;
    case 'o':
        selectedSensor = BENCH_SENSOR_TAPE5;
        BenchPrintSelectedSensor();
        return TRUE;
    case 'f':
        selectedSensor = BENCH_SENSOR_BUMP_FR;
        BenchPrintSelectedSensor();
        return TRUE;
    case 'g':
        selectedSensor = BENCH_SENSOR_BUMP_FL;
        BenchPrintSelectedSensor();
        return TRUE;
    case 'h':
        selectedSensor = BENCH_SENSOR_BUMP_RR;
        BenchPrintSelectedSensor();
        return TRUE;
    case 'j':
        selectedSensor = BENCH_SENSOR_BUMP_RL;
        BenchPrintSelectedSensor();
        return TRUE;
    case 'b':
        selectedSensor = BENCH_SENSOR_BEACON;
        BenchPrintSelectedSensor();
        return TRUE;
    default:
        return FALSE;
    }
}

static void BenchHandleMotorKey(char key)
{
    switch (key) {
    case 'w':
        RobotMotion_Forward(benchMotorSpeedIPS);
        BenchPrintMotorCommand("forward");
        break;
    case 's':
        RobotMotion_Reverse(benchMotorSpeedIPS);
        BenchPrintMotorCommand("reverse");
        break;
    case 'a':
        RobotMotion_StrafeLeft(benchMotorSpeedIPS);
        BenchPrintMotorCommand("strafe left");
        break;
    case 'd':
        RobotMotion_StrafeRight(benchMotorSpeedIPS);
        BenchPrintMotorCommand("strafe right");
        break;
    case 'q':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("turn left about center");
        break;
    case 'e':
        RobotMotion_TurnRightAbout(TURN_PIVOT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("turn right about center");
        break;
    case '1':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_FRONT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("turn left about front center");
        break;
    case '2':
        RobotMotion_TurnRightAbout(TURN_PIVOT_FRONT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("turn right about front center");
        break;
    case '3':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_BACK_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("turn left about back center");
        break;
    case '4':
        RobotMotion_TurnRightAbout(TURN_PIVOT_BACK_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("turn right about back center");
        break;
    case '5':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_LEFT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("turn left about left center");
        break;
    case '6':
        RobotMotion_TurnRightAbout(TURN_PIVOT_LEFT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("turn right about left center");
        break;
    case '7':
        RobotMotion_TurnLeftAbout(TURN_PIVOT_RIGHT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("turn left about right center");
        break;
    case '8':
        RobotMotion_TurnRightAbout(TURN_PIVOT_RIGHT_CENTER, benchMotorSpeedIPS);
        BenchPrintMotorCommand("turn right about right center");
        break;
    case 'x':
        RobotHardware_StopAllOutputs();
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
    case '+':
    case '=':
        benchMotorSpeedIPS += 1.0f;
        BenchPrintMotorCommand("speed up");
        break;
    case '-':
    case '_':
        if (benchMotorSpeedIPS > 1.0f) {
            benchMotorSpeedIPS -= 1.0f;
        }
        BenchPrintMotorCommand("speed down");
        break;
    case 'r':
        benchMotorSpeedIPS = MOTOR_SPEED_IPS;
        BenchPrintMotorCommand("reset speed");
        break;
    case 'n':
        BenchRunMotorSequenceStep();
        break;
    default:
        printf("[BENCH] key '%c' is not bound. Press ? for help.\r\n", key);
        break;
    }
}

static void BenchPrintMotorCommand(const char *commandName)
{
    printf("[MOTOR] %s at %u in/s\r\n", commandName,
            (unsigned int) benchMotorSpeedIPS);
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
        RobotMotion_StrafeRight(benchMotorSpeedIPS);
        BenchPrintMotorCommand("demo: strafe right");
        break;
    case 3u:
        RobotMotion_StrafeLeft(benchMotorSpeedIPS);
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

static void BenchPrintSelectedSensor(void)
{
    switch (selectedSensor) {
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
    case BENCH_SENSOR_ALL:
        BenchPrintTape(TAPE_SENSOR_1, "tape1", TAPE_SENSOR_1_PIN_LABEL);
        BenchPrintTape(TAPE_SENSOR_2, "tape2", TAPE_SENSOR_2_PIN_LABEL);
        BenchPrintTape(TAPE_SENSOR_3, "tape3", TAPE_SENSOR_3_PIN_LABEL);
        BenchPrintTape(TAPE_SENSOR_4, "tape4", TAPE_SENSOR_4_PIN_LABEL);
        BenchPrintTape(TAPE_SENSOR_5, "tape5", TAPE_SENSOR_5_PIN_LABEL);
        BenchPrintBump(BUMP_SENSOR_1, "bumpFR", BUMP_SENSOR_1_PIN_LABEL);
        BenchPrintBump(BUMP_SENSOR_2, "bumpFL", BUMP_SENSOR_2_PIN_LABEL);
        BenchPrintBump(BUMP_SENSOR_3, "bumpRR", BUMP_SENSOR_3_PIN_LABEL);
        BenchPrintBump(BUMP_SENSOR_4, "bumpRL", BUMP_SENSOR_4_PIN_LABEL);
        BenchPrintBeacon();
        break;
    default:
        printf("[SENSOR] none selected\r\n");
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
        RobotMotion_StrafeLeft(motorTestSpeedIPS);
        PrintMotorCommand("strafe left");
        break;
    case 'd':
        RobotMotion_StrafeRight(motorTestSpeedIPS);
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
    printf("[MOTOR] + speed up, - speed down, r reset speed, n next demo command\r\n");
    PrintMotorCommand("current speed");
    printf("\r\n");
}

static void PrintMotorCommand(const char *commandName)
{
    printf("[MOTOR] %s at %u in/s\r\n", commandName,
            (unsigned int) motorTestSpeedIPS);
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
        RobotMotion_StrafeRight(motorTestSpeedIPS);
        PrintMotorCommand("demo: strafe right");
        break;
    case 3u:
        RobotMotion_StrafeLeft(motorTestSpeedIPS);
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
