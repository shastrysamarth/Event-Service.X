/****************************************************************************
 Module
     ES_Configure.h
 Description
     Events and Services configuration for the Sluggers Lost Goal robot.
 ****************************************************************************/

#ifndef CONFIGURE_H
#define CONFIGURE_H

#ifdef ROBOT_TRACE
#define USE_TATTLETALE
#endif

typedef enum {
    /* key '0' */ ES_NO_EVENT,
    /* key '1' */ ES_ERROR,
    /* key '2' */ ES_INIT,
    /* key '3' */ ES_ENTRY,
    /* key '4' */ ES_EXIT,
    /* key '5' */ ES_KEYINPUT,
    /* key '6' */ ES_LISTEVENTS,
    /* key '7' */ ES_TIMEOUT,
    /* key '8' */ ES_TIMERACTIVE,
    /* key '9' */ ES_TIMERSTOPPED,

    /* key 'a' */ BeaconADCIncreaseEvent,
    /* key 'b' */ MaxSignalFoundEvent,

    /* Masked sensor events. EventParam packs a current mask plus a changed mask
     * (see TAPE_EVENT_* / BUMP_EVENT_* in RobotPins.h). These replace the
     * deprecated per-sensor On/Off/combo tape and bump events. */
    /* key 'c' */ TapeChangedEvent,
    /* key 'd' */ BumpChangedEvent,

    /* key 'e' */ Solenoid1OnEvent,
    /* key 'f' */ Solenoid2OnEvent,
    /* key 'g' */ Solenoid3OnEvent,
    /* key 'h' */ Solenoid4OnEvent,
    /* key 'i' */ Solenoid5OnEvent,
    /* key 'j' */ Solenoid6OnEvent,

    /* key 'k' */ MisalignedEvent,
    /* key 'l' */ PositionRealignedEvent, /* DEPRECATED: position-align removed; retained for harness / legacy. */
    /* key 'm' */ RealignedEvent,
    /* key 'n' */ DistanceMoveCompleteEvent,

    /* key 'o' */ FoundFrontTapeEvent,
    /* key 'p' */ ReachedISZEvent,
    /* key 'q' */ InsideISZEvent,
    /* key 'r' */ SetEvent,
    /* key 's' */ DoneEvent,

    NUMBEROFEVENTS,
} ES_EventTyp_t;

extern const char * const EventNames[];

#define EVENT_CHECK_HEADER "RobotEventCheckers.h"
#define EVENT_CHECK_LIST CheckBeaconEvents, CheckTapeEvents, CheckBumpEvents, \
    CheckDistanceMove, CheckSolenoidEvents, CheckAlignEvents, \
    CheckMisalignment, CheckRobotPeriodic

#define TIMER_UNUSED ((pPostFunc)0)
#define TIMER0_RESP_FUNC PostRobotHSM
#define TIMER1_RESP_FUNC PostRobotHSM
#define TIMER2_RESP_FUNC PostRobotHSM
#define TIMER3_RESP_FUNC TIMER_UNUSED
#define TIMER4_RESP_FUNC TIMER_UNUSED
#define TIMER5_RESP_FUNC TIMER_UNUSED
#define TIMER6_RESP_FUNC TIMER_UNUSED
#define TIMER7_RESP_FUNC TIMER_UNUSED
#define TIMER8_RESP_FUNC TIMER_UNUSED
#define TIMER9_RESP_FUNC TIMER_UNUSED
#define TIMER10_RESP_FUNC TIMER_UNUSED
#define TIMER11_RESP_FUNC TIMER_UNUSED
#define TIMER12_RESP_FUNC TIMER_UNUSED
#define TIMER13_RESP_FUNC TIMER_UNUSED
#define TIMER14_RESP_FUNC TIMER_UNUSED
#define TIMER15_RESP_FUNC TIMER_UNUSED

#define MAX_NUM_SERVICES 8
#define NUM_SERVICES 1

#define SERV_0_HEADER "RobotHSM.h"
#define SERV_0_INIT InitRobotHSM
#define SERV_0_RUN RunRobotHSM
#define SERV_0_QUEUE_SIZE 32

#if NUM_SERVICES > 1
#define SERV_1_HEADER "RobotHSM.h"
#define SERV_1_INIT InitRobotHSM
#define SERV_1_RUN RunRobotHSM
#define SERV_1_QUEUE_SIZE 3
#endif

#if NUM_SERVICES > 2
#define SERV_2_HEADER "RobotHSM.h"
#define SERV_2_INIT InitRobotHSM
#define SERV_2_RUN RunRobotHSM
#define SERV_2_QUEUE_SIZE 3
#endif

#if NUM_SERVICES > 3
#define SERV_3_HEADER "RobotHSM.h"
#define SERV_3_INIT InitRobotHSM
#define SERV_3_RUN RunRobotHSM
#define SERV_3_QUEUE_SIZE 3
#endif

#if NUM_SERVICES > 4
#define SERV_4_HEADER "RobotHSM.h"
#define SERV_4_INIT InitRobotHSM
#define SERV_4_RUN RunRobotHSM
#define SERV_4_QUEUE_SIZE 3
#endif

#if NUM_SERVICES > 5
#define SERV_5_HEADER "RobotHSM.h"
#define SERV_5_INIT InitRobotHSM
#define SERV_5_RUN RunRobotHSM
#define SERV_5_QUEUE_SIZE 3
#endif

#if NUM_SERVICES > 6
#define SERV_6_HEADER "RobotHSM.h"
#define SERV_6_INIT InitRobotHSM
#define SERV_6_RUN RunRobotHSM
#define SERV_6_QUEUE_SIZE 3
#endif

#if NUM_SERVICES > 7
#define SERV_7_HEADER "RobotHSM.h"
#define SERV_7_INIT InitRobotHSM
#define SERV_7_RUN RunRobotHSM
#define SERV_7_QUEUE_SIZE 3
#endif

#define POST_KEY_FUNC ES_PostAll
#define NUM_DIST_LISTS 0

#endif /* CONFIGURE_H */
