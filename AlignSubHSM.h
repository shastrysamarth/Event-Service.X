#ifndef ALIGN_SUB_HSM_H
#define ALIGN_SUB_HSM_H

#include "BOARD.h"
#include "ES_Configure.h"
#include "ES_Events.h"
#include "RobotTypes.h"

typedef enum {
    ALIGN_MODE_GYRO,
    ALIGN_MODE_TAPE,
} AlignMode_t;

uint8_t InitAlignSubHSM(MovementAxis_t axis, float xRefInches, float yRefInches);
/* pivot selects the turn center for the granular gyro correction (e.g. pivot
 * about the tape sensor the caller is tracking). */
uint8_t InitGyroAlignSubHSM(MovementAxis_t axis, TurnPivot_t pivot,
        float xRefInches, float yRefInches);
uint8_t InitTapeAlignSubHSM(MovementAxis_t axis, BoundaryChoice_t boundary,
        float xRefInches, float yRefInches);
ES_Event RunAlignSubHSM(ES_Event ThisEvent);

uint8_t AlignSubHSM_IsActive(void);
uint8_t AlignSubHSM_IsPositionStage(void);
uint8_t AlignSubHSM_IsHeadingStage(void);
uint8_t AlignSubHSM_IsTapeStage(void);
void AlignSubHSM_UpdateControl(void);
uint8_t AlignSubHSM_IsPositionAligned(void);
uint8_t AlignSubHSM_IsHeadingAligned(void);
const char *AlignSubHSM_GetStateName(void);
MovementAxis_t AlignSubHSM_GetAxis(void);
AlignMode_t AlignSubHSM_GetMode(void);
float AlignSubHSM_GetXRef(void);
float AlignSubHSM_GetYRef(void);
float AlignSubHSM_GetPositionError(void);

#endif /* ALIGN_SUB_HSM_H */
