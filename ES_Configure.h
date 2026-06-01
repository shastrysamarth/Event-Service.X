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

    /* key 'n' */ TapeSensor1And2OffEvent,
    /* key 'o' */ TapeSensor1And5OffEvent,
    /* key 'p' */ TapeSensor2And5OffEvent,
    /* key 'q' */ TapeSensor3And4OffEvent,
    /* key 'r' */ TapeSensor3And5OffEvent,
    /* key 's' */ TapeSensor4And5OffEvent,

    /* key 't' */ Solenoid1OnEvent,
    /* key 'u' */ Solenoid2OnEvent,
    /* key 'v' */ Solenoid3OnEvent,
    /* key 'w' */ Solenoid4OnEvent,
    /* key 'x' */ Solenoid5OnEvent,
    /* key 'y' */ Solenoid6OnEvent,

    /* key 'z' */ BumpSensor1OnEvent,
    /* key 'A' */ BumpSensor1OffEvent,
    /* key 'B' */ BumpSensor2OnEvent,
    /* key 'C' */ BumpSensor2OffEvent,
    /* key 'D' */ BumpSensor3OnEvent,
    /* key 'E' */ BumpSensor3OffEvent,
    /* key 'F' */ BumpSensor4OnEvent,
    /* key 'G' */ BumpSensor4OffEvent,

    /* key 'H' */ MisalignedEvent,
    /* key 'I' */ PositionRealignedEvent, /* DEPRECATED: position-align removed; retained for harness / legacy. */
    /* key 'J' */ RealignedEvent,
    /* key 'K' */ DistanceMoveCompleteEvent,

    /* key 'L' */ FoundFrontTapeEvent,
    /* key 'M' */ ReachedISZEvent,
    /* key 'N' */ InsideISZEvent,
    /* key 'O' */ SetEvent,
    /* key 'P' */ DoneEvent,

    NUMBEROFEVENTS,
} ES_EventTyp_t;

extern const char * const EventNames[];

#define EVENT_CHECK_HEADER "RobotEventCheckers.h"
#define EVENT_CHECK_LIST CheckBeaconSample, CheckTapeEvents, CheckBumpEvents, \
    CheckDistanceMove, CheckRobotPeriodic, CheckBeaconEvents, CheckSolenoidEvents, \
    CheckAlignEvents, CheckMisalignment

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
