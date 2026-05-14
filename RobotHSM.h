#ifndef ROBOT_HSM_H
#define ROBOT_HSM_H

#include "BOARD.h"
#include "ES_Configure.h"
#include "ES_Events.h"

uint8_t InitRobotHSM(uint8_t priority);
uint8_t PostRobotHSM(ES_Event ThisEvent);
ES_Event RunRobotHSM(ES_Event ThisEvent);
const char *RobotHSM_GetStateName(void);

#endif /* ROBOT_HSM_H */
