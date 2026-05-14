#include "RobotTestHarness.h"

#include "ES_Configure.h"
#include "ES_Events.h"
#include "RobotDebug.h"
#include "RobotHSM.h"
#include "RobotIMU.h"
#include "RobotPins.h"
#include "RobotPlugPlay.h"
#include "serial.h"

#include <stdio.h>

#ifdef ROBOT_KEYBOARD_TEST
static uint8_t PostEventByType(ES_EventTyp_t eventType, uint16_t eventParam,
        const char *sourceLabel);
static uint8_t GetEventForKey(char key, ES_EventTyp_t *eventType);
static char GetKeyForEventIndex(uint8_t eventIndex);
static uint16_t GetDefaultParamForEvent(ES_EventTyp_t eventType);
static void PrintBindingForKey(char key);

static uint8_t queryNextKey = FALSE;

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
    printf("[TEST] key ranges: 0-9, a-z, A-Z in EventNames[] order\r\n");
    printf("\r\n");
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
