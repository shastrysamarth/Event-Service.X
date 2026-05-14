#ifndef FIND_FRONT_TAPE_SUB_HSM_H
#define FIND_FRONT_TAPE_SUB_HSM_H

#include "BOARD.h"
#include "ES_Configure.h"
#include "ES_Events.h"
#include "RobotTypes.h"

uint8_t InitFindFrontTapeSubHSM(void);
ES_Event RunFindFrontTapeSubHSM(ES_Event ThisEvent);
BoundaryChoice_t FindFrontTape_GetBoundaryChoice(void);
uint8_t FindFrontTape_IsBeaconSearchActive(void);
const char *FindFrontTape_GetStateName(void);

#endif /* FIND_FRONT_TAPE_SUB_HSM_H */
