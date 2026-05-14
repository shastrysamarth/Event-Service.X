#ifndef ALIGN_SUB_HSM_H
#define ALIGN_SUB_HSM_H

/*
 * Heading-only align is active. Position stage, PositionRealignedEvent, xRef/yRef
 * position error APIs are DEPRECATED but kept for compatibility and test harness.
 */

#include "BOARD.h"
#include "ES_Configure.h"
#include "ES_Events.h"
#include "RobotTypes.h"

uint8_t InitAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches);
ES_Event RunAlignSubHSM(ES_Event ThisEvent);

uint8_t AlignSubHSM_IsActive(void);
/* DEPRECATED: position stage no longer entered; always false in normal flow. */
uint8_t AlignSubHSM_IsPositionStage(void);
uint8_t AlignSubHSM_IsHeadingStage(void);
void AlignSubHSM_UpdateControl(void);
/* DEPRECATED: IMU position vs ref no longer drives align completion. */
uint8_t AlignSubHSM_IsPositionAligned(void);
uint8_t AlignSubHSM_IsHeadingAligned(void);
const char *AlignSubHSM_GetStateName(void);
MovementAxis_t AlignSubHSM_GetAxis(void);
/* DEPRECATED: refs kept for debug / Navigate bookkeeping only. */
float AlignSubHSM_GetXRef(void);
float AlignSubHSM_GetYRef(void);
/* DEPRECATED */
float AlignSubHSM_GetPositionError(void);

#endif /* ALIGN_SUB_HSM_H */
