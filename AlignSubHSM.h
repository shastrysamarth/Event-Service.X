#ifndef ALIGN_SUB_HSM_H
#define ALIGN_SUB_HSM_H

#include "BOARD.h"
#include "ES_Configure.h"
#include "ES_Events.h"
#include "RobotTypes.h"

uint8_t InitAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches);
ES_Event RunAlignSubHSM(ES_Event ThisEvent);

uint8_t AlignSubHSM_IsActive(void);
uint8_t AlignSubHSM_IsPositionStage(void);
uint8_t AlignSubHSM_IsHeadingStage(void);
void AlignSubHSM_UpdateControl(void);
uint8_t AlignSubHSM_IsPositionAligned(void);
uint8_t AlignSubHSM_IsHeadingAligned(void);
const char *AlignSubHSM_GetStateName(void);
MovementAxis_t AlignSubHSM_GetAxis(void);
float AlignSubHSM_GetXRef(void);
float AlignSubHSM_GetYRef(void);
float AlignSubHSM_GetPositionError(void);

#endif /* ALIGN_SUB_HSM_H */
