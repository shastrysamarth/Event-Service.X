#ifndef NAVIGATE_TO_ISZ_SUB_HSM_H
#define NAVIGATE_TO_ISZ_SUB_HSM_H

#include "BOARD.h"
#include "ES_Configure.h"
#include "ES_Events.h"
#include "RobotTypes.h"

uint8_t InitNavigateToISZSubHSM(BoundaryChoice_t startingBoundary);
ES_Event RunNavigateToISZSubHSM(ES_Event ThisEvent);

uint8_t NavigateToISZ_IsActive(void);
uint8_t NavigateToISZ_IsAligning(void);
uint8_t NavigateToISZ_AllowsAlign(void);
uint8_t NavigateToISZ_IsCountingTape5(void);
MovementAxis_t NavigateToISZ_GetMovementAxis(void);
float NavigateToISZ_GetXRef(void);
float NavigateToISZ_GetYRef(void);
const char *NavigateToISZ_GetStateName(void);
BoundaryChoice_t NavigateToISZ_GetBoundaryChoice(void);
uint8_t NavigateToISZ_GetNumTapesCrossed(void);

#endif /* NAVIGATE_TO_ISZ_SUB_HSM_H */
