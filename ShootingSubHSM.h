#ifndef SHOOTING_SUB_HSM_H
#define SHOOTING_SUB_HSM_H

#include "BOARD.h"
#include "ES_Configure.h"
#include "ES_Events.h"

uint8_t InitShootingSubHSM(void);
ES_Event RunShootingSubHSM(ES_Event ThisEvent);
uint8_t ShootingSubHSM_IsBeaconSearchActive(void);
const char *ShootingSubHSM_GetStateName(void);
uint16_t ShootingSubHSM_GetMaxBeaconADC(void);

#endif /* SHOOTING_SUB_HSM_H */
