#include <BOARD.h>
#include <stdio.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "RobotHardware.h"
#include "RobotPlugPlay.h"
#include "RobotTestHarness.h"

void main(void)
{
#if !defined(ROBOT_MOTOR_SENSOR_TEST) && !defined(ROBOT_BEACON_TEST) && \
    !defined(ROBOT_GPIO_HIGH_TEST)
    ES_Return_t errorType;
#endif

    BOARD_Init();
    RobotHardware_Init();

#ifdef ROBOT_MOTOR_SENSOR_TEST
    printf("Starting direct motor/sensor bench harness\r\n");
    RobotPlugPlay_PrintConfig();
    RobotTestHarness_RunMotorSensorBench();
#elif defined(ROBOT_BEACON_TEST)
    printf("Starting direct beacon ADC bench harness\r\n");
    RobotPlugPlay_PrintConfig();
    RobotTestHarness_RunBeaconBench();
#elif defined(ROBOT_GPIO_HIGH_TEST)
    printf("Starting GPIO-high bench harness\r\n");
    RobotPlugPlay_PrintConfig();
    RobotTestHarness_RunGPIOHighBench();
#else
    printf("Starting Sluggers Lost Goal full HSM\r\n");
#ifdef ROBOT_KEYBOARD_TEST
    printf("Keyboard HSM test harness enabled\r\n");
    RobotTestHarness_PrintHelp();
#endif
    RobotPlugPlay_PrintConfig();

    errorType = ES_Initialize();
    if (errorType == Success) {
        errorType = ES_Run();
    }

    RobotHardware_StopAllOutputs();

    switch (errorType) {
    case FailedPointer:
        printf("Failed on NULL pointer\r\n");
        break;
    case FailedInit:
        printf("Failed initialization\r\n");
        break;
    default:
        printf("Framework failure: %d\r\n", errorType);
        break;
    }

    for (;;) {
    }
#endif
}
