#include "RobotHardware.h"

#include "RobotIMU.h"
#include "RobotLauncher.h"
#include "RobotMotion.h"
#include "RobotSensors.h"
#include "RobotStepper.h"

uint8_t RobotHardware_Init(void)
{
    uint8_t ok = TRUE;

    if (RobotSensors_Init() != TRUE) {
        ok = FALSE;
    }
    if (RobotMotion_Init() != TRUE) {
        ok = FALSE;
    }
    if (RobotLauncher_Init() != TRUE) {
        ok = FALSE;
    }
    if (RobotIMU_Init() != TRUE) {
        ok = FALSE;
    }
    if (RobotStepper_Init() != TRUE) {
        ok = FALSE;
    }

    RobotHardware_StopAllOutputs();
    return ok;
}

void RobotHardware_StopAllOutputs(void)
{
    RobotMotion_Stop();
    RobotLauncher_StopShooter();
    RobotStepper_Disable();
}
