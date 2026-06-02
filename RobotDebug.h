#ifndef ROBOT_DEBUG_H
#define ROBOT_DEBUG_H

#include "ES_Configure.h"
#include "ES_Events.h"
#include "RobotPins.h"

void RobotDebug_LogStateEntry(const char *machineName,
        const char *stateName, ES_Event event);
void RobotDebug_PrintCurrentState(void);
void RobotDebug_PrintModuleVariables(void);

#if (defined(DEBUG) || defined(ROBOT_DEBUG)) && ROBOT_LOG_STATE
#define ROBOT_DEBUG_STATE(machineName, stateName, event) \
    RobotDebug_LogStateEntry((machineName), (stateName), (event))
#else
#define ROBOT_DEBUG_STATE(machineName, stateName, event) ((void) 0)
#endif

#endif /* ROBOT_DEBUG_H */
