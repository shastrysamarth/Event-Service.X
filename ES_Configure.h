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

    /* key 'c' */ TapeSensor1OnEvent,
    /* key 'd' */ TapeSensor1OffEvent,
    /* key 'e' */ TapeSensor2OnEvent,
    /* key 'f' */ TapeSensor2OffEvent,
    /* key 'g' */ TapeSensor3OnEvent,
    /* key 'h' */ TapeSensor3OffEvent,
    /* key 'i' */ TapeSensor4OnEvent,
    /* key 'j' */ TapeSensor4OffEvent,
    /* key 'k' */ TapeSensor5OnEvent,
    /* key 'l' */ TapeSensor5OffEvent,
    /* key 'm' */ TapeSensor5LowToHighEvent,

    /* key 'n' */ TapeSensor2On3OffEvent,
    /* key 'o' */ TapeSensor2On4OffEvent,
    /* key 'p' */ TapeSensor2Off3OnEvent,
    /* key 'q' */ TapeSensor2Off4OnEvent,

    /* key 'r' */ Solenoid1OnEvent,
    /* key 's' */ Solenoid2OnEvent,
    /* key 't' */ Solenoid3OnEvent,
    /* key 'u' */ Solenoid4OnEvent,
    /* key 'v' */ Solenoid5OnEvent,
    /* key 'w' */ Solenoid6OnEvent,

    /* key 'x' */ BumpSensor1OnEvent,
    /* key 'y' */ BumpSensor1OffEvent,
    /* key 'z' */ BumpSensor2OnEvent,

    /* key 'A' */ BumpSensor2OffEvent,
    /* key 'B' */ BumpSensor3OnEvent,
    /* key 'C' */ BumpSensor3OffEvent,
    /* key 'D' */ BumpSensor4OnEvent,
    /* key 'E' */ BumpSensor4OffEvent,

    /* key 'F' */ MisalignedEvent,
    /* key 'G' */ PositionRealignedEvent, /* DEPRECATED: position-align removed; retained for harness / legacy. */
    /* key 'H' */ RealignedEvent,
    /* key 'I' */ DistanceMoveCompleteEvent,

    /* key 'J' */ FoundFrontTapeEvent,
    /* key 'K' */ ReachedISZEvent,
    /* key 'L' */ InsideISZEvent,
    /* key 'M' */ SetEvent,
    /* key 'N' */ DoneEvent,

    NUMBEROFEVENTS,
} ES_EventTyp_t;

static const char *EventNames[] = {
    "ES_NO_EVENT",
    "ES_ERROR",
    "ES_INIT",
    "ES_ENTRY",
    "ES_EXIT",
    "ES_KEYINPUT",
    "ES_LISTEVENTS",
    "ES_TIMEOUT",
    "ES_TIMERACTIVE",
    "ES_TIMERSTOPPED",
    "BeaconADCIncreaseEvent",
    "MaxSignalFoundEvent",
    "TapeSensor1OnEvent",
    "TapeSensor1OffEvent",
    "TapeSensor2OnEvent",
    "TapeSensor2OffEvent",
    "TapeSensor3OnEvent",
    "TapeSensor3OffEvent",
    "TapeSensor4OnEvent",
    "TapeSensor4OffEvent",
    "TapeSensor5OnEvent",
    "TapeSensor5OffEvent",
    "TapeSensor5LowToHighEvent",
    "TapeSensor2On3OffEvent",
    "TapeSensor2On4OffEvent",
    "TapeSensor2Off3OnEvent",
    "TapeSensor2Off4OnEvent",
    "Solenoid1OnEvent",
    "Solenoid2OnEvent",
    "Solenoid3OnEvent",
    "Solenoid4OnEvent",
    "Solenoid5OnEvent",
    "Solenoid6OnEvent",
    "BumpSensor1OnEvent",
    "BumpSensor1OffEvent",
    "BumpSensor2OnEvent",
    "BumpSensor2OffEvent",
    "BumpSensor3OnEvent",
    "BumpSensor3OffEvent",
    "BumpSensor4OnEvent",
    "BumpSensor4OffEvent",
    "MisalignedEvent",
    "PositionRealignedEvent",
    "RealignedEvent",
    "DistanceMoveCompleteEvent",
    "FoundFrontTapeEvent",
    "ReachedISZEvent",
    "InsideISZEvent",
    "SetEvent",
    "DoneEvent",
    "NUMBEROFEVENTS",
};

#define EVENT_CHECK_HEADER "RobotEventCheckers.h"
#define EVENT_CHECK_LIST CheckRobotPeriodic, CheckBeaconEvents, \
    CheckCompoundNavigationEvents, CheckTapeEvents, CheckSolenoidEvents, \
    CheckBumpEvents, CheckDistanceMove, CheckAlignEvents, CheckMisalignment

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
