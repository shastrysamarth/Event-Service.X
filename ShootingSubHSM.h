#ifndef SHOOTING_SUB_HSM_H
#define SHOOTING_SUB_HSM_H

#include "BOARD.h"
#include "ES_Configure.h"
#include "ES_Events.h"
#include "RobotTypes.h"

uint8_t InitShootingSubHSM(BoundaryChoice_t startingBoundary);
ES_Event RunShootingSubHSM(ES_Event ThisEvent);
uint8_t ShootingSubHSM_IsBeaconSearchActive(void);
uint8_t ShootingSubHSM_IsAligning(void);
uint8_t ShootingSubHSM_AllowsAlign(void);
const char *ShootingSubHSM_GetStateName(void);
uint16_t ShootingSubHSM_GetMaxBeaconADC(void);

#endif /* SHOOTING_SUB_HSM_H */
