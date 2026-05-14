#include <BOARD.h>
#include <stdio.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "RobotHardware.h"
#include "RobotPlugPlay.h"
#include "RobotTestHarness.h"

void main(void)
{
    ES_Return_t errorType;

    BOARD_Init();
    RobotHardware_Init();

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
}
