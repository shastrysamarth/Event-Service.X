#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/Event_Service.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/Event_Service.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=/Users/king/VSCode/ECE118/src/AD.c /Users/king/VSCode/ECE118/src/BOARD.c /Users/king/VSCode/ECE118/src/ES_CheckEvents.c /Users/king/VSCode/ECE118/src/ES_Framework.c /Users/king/VSCode/ECE118/src/ES_KeyboardInput.c /Users/king/VSCode/ECE118/src/ES_PostList.c /Users/king/VSCode/ECE118/src/ES_Queue.c /Users/king/VSCode/ECE118/src/ES_TattleTale.c /Users/king/VSCode/ECE118/src/ES_Timers.c /Users/king/VSCode/ECE118/src/IO_Ports.c /Users/king/VSCode/ECE118/src/LED.c /Users/king/VSCode/ECE118/src/pwm.c /Users/king/VSCode/ECE118/src/RC_Servo.c /Users/king/VSCode/ECE118/src/roach.c /Users/king/VSCode/ECE118/src/serial.c /Users/king/VSCode/ECE118/src/timers.c "/Users/king/MPLABXProjects/Event Service.X/AlignSubHSM.c" "/Users/king/MPLABXProjects/Event Service.X/FindFrontTapeSubHSM.c" "/Users/king/MPLABXProjects/Event Service.X/main.c" "/Users/king/MPLABXProjects/Event Service.X/NavigateToISZSubHSM.c" "/Users/king/MPLABXProjects/Event Service.X/RobotDebug.c" "/Users/king/MPLABXProjects/Event Service.X/RobotEventCheckers.c" "/Users/king/MPLABXProjects/Event Service.X/RobotHardware.c" "/Users/king/MPLABXProjects/Event Service.X/RobotHSM.c" "/Users/king/MPLABXProjects/Event Service.X/RobotIMU.c" "/Users/king/MPLABXProjects/Event Service.X/RobotLauncher.c" "/Users/king/MPLABXProjects/Event Service.X/RobotMotion.c" "/Users/king/MPLABXProjects/Event Service.X/RobotPlugPlay.c" "/Users/king/MPLABXProjects/Event Service.X/RobotSensors.c" "/Users/king/MPLABXProjects/Event Service.X/RobotStepper.c" "/Users/king/MPLABXProjects/Event Service.X/RobotTestHarness.c" "/Users/king/MPLABXProjects/Event Service.X/ShootingSubHSM.c"

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/830340023/AD.o ${OBJECTDIR}/_ext/830340023/BOARD.o ${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o ${OBJECTDIR}/_ext/830340023/ES_Framework.o ${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o ${OBJECTDIR}/_ext/830340023/ES_PostList.o ${OBJECTDIR}/_ext/830340023/ES_Queue.o ${OBJECTDIR}/_ext/830340023/ES_TattleTale.o ${OBJECTDIR}/_ext/830340023/ES_Timers.o ${OBJECTDIR}/_ext/830340023/IO_Ports.o ${OBJECTDIR}/_ext/830340023/LED.o ${OBJECTDIR}/_ext/830340023/pwm.o ${OBJECTDIR}/_ext/830340023/RC_Servo.o ${OBJECTDIR}/_ext/830340023/roach.o ${OBJECTDIR}/_ext/830340023/serial.o ${OBJECTDIR}/_ext/830340023/timers.o ${OBJECTDIR}/_ext/118733356/AlignSubHSM.o ${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o ${OBJECTDIR}/_ext/118733356/main.o ${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o ${OBJECTDIR}/_ext/118733356/RobotDebug.o ${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o ${OBJECTDIR}/_ext/118733356/RobotHardware.o ${OBJECTDIR}/_ext/118733356/RobotHSM.o ${OBJECTDIR}/_ext/118733356/RobotIMU.o ${OBJECTDIR}/_ext/118733356/RobotLauncher.o ${OBJECTDIR}/_ext/118733356/RobotMotion.o ${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o ${OBJECTDIR}/_ext/118733356/RobotSensors.o ${OBJECTDIR}/_ext/118733356/RobotStepper.o ${OBJECTDIR}/_ext/118733356/RobotTestHarness.o ${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/830340023/AD.o.d ${OBJECTDIR}/_ext/830340023/BOARD.o.d ${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o.d ${OBJECTDIR}/_ext/830340023/ES_Framework.o.d ${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o.d ${OBJECTDIR}/_ext/830340023/ES_PostList.o.d ${OBJECTDIR}/_ext/830340023/ES_Queue.o.d ${OBJECTDIR}/_ext/830340023/ES_TattleTale.o.d ${OBJECTDIR}/_ext/830340023/ES_Timers.o.d ${OBJECTDIR}/_ext/830340023/IO_Ports.o.d ${OBJECTDIR}/_ext/830340023/LED.o.d ${OBJECTDIR}/_ext/830340023/pwm.o.d ${OBJECTDIR}/_ext/830340023/RC_Servo.o.d ${OBJECTDIR}/_ext/830340023/roach.o.d ${OBJECTDIR}/_ext/830340023/serial.o.d ${OBJECTDIR}/_ext/830340023/timers.o.d ${OBJECTDIR}/_ext/118733356/AlignSubHSM.o.d ${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o.d ${OBJECTDIR}/_ext/118733356/main.o.d ${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o.d ${OBJECTDIR}/_ext/118733356/RobotDebug.o.d ${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o.d ${OBJECTDIR}/_ext/118733356/RobotHardware.o.d ${OBJECTDIR}/_ext/118733356/RobotHSM.o.d ${OBJECTDIR}/_ext/118733356/RobotIMU.o.d ${OBJECTDIR}/_ext/118733356/RobotLauncher.o.d ${OBJECTDIR}/_ext/118733356/RobotMotion.o.d ${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o.d ${OBJECTDIR}/_ext/118733356/RobotSensors.o.d ${OBJECTDIR}/_ext/118733356/RobotStepper.o.d ${OBJECTDIR}/_ext/118733356/RobotTestHarness.o.d ${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/830340023/AD.o ${OBJECTDIR}/_ext/830340023/BOARD.o ${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o ${OBJECTDIR}/_ext/830340023/ES_Framework.o ${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o ${OBJECTDIR}/_ext/830340023/ES_PostList.o ${OBJECTDIR}/_ext/830340023/ES_Queue.o ${OBJECTDIR}/_ext/830340023/ES_TattleTale.o ${OBJECTDIR}/_ext/830340023/ES_Timers.o ${OBJECTDIR}/_ext/830340023/IO_Ports.o ${OBJECTDIR}/_ext/830340023/LED.o ${OBJECTDIR}/_ext/830340023/pwm.o ${OBJECTDIR}/_ext/830340023/RC_Servo.o ${OBJECTDIR}/_ext/830340023/roach.o ${OBJECTDIR}/_ext/830340023/serial.o ${OBJECTDIR}/_ext/830340023/timers.o ${OBJECTDIR}/_ext/118733356/AlignSubHSM.o ${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o ${OBJECTDIR}/_ext/118733356/main.o ${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o ${OBJECTDIR}/_ext/118733356/RobotDebug.o ${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o ${OBJECTDIR}/_ext/118733356/RobotHardware.o ${OBJECTDIR}/_ext/118733356/RobotHSM.o ${OBJECTDIR}/_ext/118733356/RobotIMU.o ${OBJECTDIR}/_ext/118733356/RobotLauncher.o ${OBJECTDIR}/_ext/118733356/RobotMotion.o ${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o ${OBJECTDIR}/_ext/118733356/RobotSensors.o ${OBJECTDIR}/_ext/118733356/RobotStepper.o ${OBJECTDIR}/_ext/118733356/RobotTestHarness.o ${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o

# Source Files
SOURCEFILES=/Users/king/VSCode/ECE118/src/AD.c /Users/king/VSCode/ECE118/src/BOARD.c /Users/king/VSCode/ECE118/src/ES_CheckEvents.c /Users/king/VSCode/ECE118/src/ES_Framework.c /Users/king/VSCode/ECE118/src/ES_KeyboardInput.c /Users/king/VSCode/ECE118/src/ES_PostList.c /Users/king/VSCode/ECE118/src/ES_Queue.c /Users/king/VSCode/ECE118/src/ES_TattleTale.c /Users/king/VSCode/ECE118/src/ES_Timers.c /Users/king/VSCode/ECE118/src/IO_Ports.c /Users/king/VSCode/ECE118/src/LED.c /Users/king/VSCode/ECE118/src/pwm.c /Users/king/VSCode/ECE118/src/RC_Servo.c /Users/king/VSCode/ECE118/src/roach.c /Users/king/VSCode/ECE118/src/serial.c /Users/king/VSCode/ECE118/src/timers.c /Users/king/MPLABXProjects/Event Service.X/AlignSubHSM.c /Users/king/MPLABXProjects/Event Service.X/FindFrontTapeSubHSM.c /Users/king/MPLABXProjects/Event Service.X/main.c /Users/king/MPLABXProjects/Event Service.X/NavigateToISZSubHSM.c /Users/king/MPLABXProjects/Event Service.X/RobotDebug.c /Users/king/MPLABXProjects/Event Service.X/RobotEventCheckers.c /Users/king/MPLABXProjects/Event Service.X/RobotHardware.c /Users/king/MPLABXProjects/Event Service.X/RobotHSM.c /Users/king/MPLABXProjects/Event Service.X/RobotIMU.c /Users/king/MPLABXProjects/Event Service.X/RobotLauncher.c /Users/king/MPLABXProjects/Event Service.X/RobotMotion.c /Users/king/MPLABXProjects/Event Service.X/RobotPlugPlay.c /Users/king/MPLABXProjects/Event Service.X/RobotSensors.c /Users/king/MPLABXProjects/Event Service.X/RobotStepper.c /Users/king/MPLABXProjects/Event Service.X/RobotTestHarness.c /Users/king/MPLABXProjects/Event Service.X/ShootingSubHSM.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/Event_Service.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX320F128H
MP_LINKER_FILE_OPTION=,--script="/Users/king/VSCode/ECE118/bootloader320.ld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/830340023/AD.o: /Users/king/VSCode/ECE118/src/AD.c  .generated_files/flags/default/59538357a539458504affc9e285bf0786db30368 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/AD.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/AD.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/AD.o.d" -o ${OBJECTDIR}/_ext/830340023/AD.o /Users/king/VSCode/ECE118/src/AD.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/BOARD.o: /Users/king/VSCode/ECE118/src/BOARD.c  .generated_files/flags/default/5993c884361c2693b410a7d51f854902329c60c0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/BOARD.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/BOARD.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/BOARD.o.d" -o ${OBJECTDIR}/_ext/830340023/BOARD.o /Users/king/VSCode/ECE118/src/BOARD.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o: /Users/king/VSCode/ECE118/src/ES_CheckEvents.c  .generated_files/flags/default/f0fd2401084331ea9801b135bfe4e10a1a2a0539 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o /Users/king/VSCode/ECE118/src/ES_CheckEvents.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_Framework.o: /Users/king/VSCode/ECE118/src/ES_Framework.c  .generated_files/flags/default/1496001cccae51188046112d0ca252eda04fbc4c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Framework.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Framework.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_Framework.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_Framework.o /Users/king/VSCode/ECE118/src/ES_Framework.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o: /Users/king/VSCode/ECE118/src/ES_KeyboardInput.c  .generated_files/flags/default/e2b4ed54d6c7a80271097a86daa21023604dd87a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o /Users/king/VSCode/ECE118/src/ES_KeyboardInput.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_PostList.o: /Users/king/VSCode/ECE118/src/ES_PostList.c  .generated_files/flags/default/ef6520545a0f482dc836ab7f6697b41e95e918f4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_PostList.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_PostList.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_PostList.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_PostList.o /Users/king/VSCode/ECE118/src/ES_PostList.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_Queue.o: /Users/king/VSCode/ECE118/src/ES_Queue.c  .generated_files/flags/default/58352c410367d02bb0809448fad6552753ca1e29 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_Queue.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_Queue.o /Users/king/VSCode/ECE118/src/ES_Queue.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_TattleTale.o: /Users/king/VSCode/ECE118/src/ES_TattleTale.c  .generated_files/flags/default/ebc99481a3b304aabb88c96cff4a9443ddcc96d2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_TattleTale.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_TattleTale.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_TattleTale.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_TattleTale.o /Users/king/VSCode/ECE118/src/ES_TattleTale.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_Timers.o: /Users/king/VSCode/ECE118/src/ES_Timers.c  .generated_files/flags/default/508ad945ca703010f8651c8c4c8be64e6c0ecf85 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_Timers.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_Timers.o /Users/king/VSCode/ECE118/src/ES_Timers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/IO_Ports.o: /Users/king/VSCode/ECE118/src/IO_Ports.c  .generated_files/flags/default/33f6b92d7646f3d4a5fee4b9027c6eeb69da5ef4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/IO_Ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/IO_Ports.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/IO_Ports.o.d" -o ${OBJECTDIR}/_ext/830340023/IO_Ports.o /Users/king/VSCode/ECE118/src/IO_Ports.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/LED.o: /Users/king/VSCode/ECE118/src/LED.c  .generated_files/flags/default/959ba3aad013e12f8ae66c8c70ba011f56353bf3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/LED.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/LED.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/LED.o.d" -o ${OBJECTDIR}/_ext/830340023/LED.o /Users/king/VSCode/ECE118/src/LED.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/pwm.o: /Users/king/VSCode/ECE118/src/pwm.c  .generated_files/flags/default/262c6df96942c3e92368dfa968d895775ea50cac .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/pwm.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/pwm.o.d" -o ${OBJECTDIR}/_ext/830340023/pwm.o /Users/king/VSCode/ECE118/src/pwm.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/RC_Servo.o: /Users/king/VSCode/ECE118/src/RC_Servo.c  .generated_files/flags/default/10e1d48b947d8f44f93141ee757795517f15c172 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/RC_Servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/RC_Servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/RC_Servo.o.d" -o ${OBJECTDIR}/_ext/830340023/RC_Servo.o /Users/king/VSCode/ECE118/src/RC_Servo.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/roach.o: /Users/king/VSCode/ECE118/src/roach.c  .generated_files/flags/default/7049ae497de4d6252546d7c99012fa45fbf25caa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/roach.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/roach.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/roach.o.d" -o ${OBJECTDIR}/_ext/830340023/roach.o /Users/king/VSCode/ECE118/src/roach.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/serial.o: /Users/king/VSCode/ECE118/src/serial.c  .generated_files/flags/default/f1016ade5404da85b1bbea5885c38b8f692ca053 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/serial.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/serial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/serial.o.d" -o ${OBJECTDIR}/_ext/830340023/serial.o /Users/king/VSCode/ECE118/src/serial.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/timers.o: /Users/king/VSCode/ECE118/src/timers.c  .generated_files/flags/default/7e6c4e84e37138e389769a73356a39ea84597d80 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/timers.o.d" -o ${OBJECTDIR}/_ext/830340023/timers.o /Users/king/VSCode/ECE118/src/timers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/AlignSubHSM.o: /Users/king/MPLABXProjects/Event\ Service.X/AlignSubHSM.c  .generated_files/flags/default/f2d53502389a380af07ff4ca0bbd79d38050e0ad .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/AlignSubHSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/AlignSubHSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/AlignSubHSM.o.d" -o ${OBJECTDIR}/_ext/118733356/AlignSubHSM.o "/Users/king/MPLABXProjects/Event Service.X/AlignSubHSM.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o: /Users/king/MPLABXProjects/Event\ Service.X/FindFrontTapeSubHSM.c  .generated_files/flags/default/110e47922c4c58e0a5ae9c4e231698e2a96f15f6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o.d" -o ${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o "/Users/king/MPLABXProjects/Event Service.X/FindFrontTapeSubHSM.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/main.o: /Users/king/MPLABXProjects/Event\ Service.X/main.c  .generated_files/flags/default/54dee9bd19f28e26a4a3540a5629eb52a377d2b7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/main.o.d" -o ${OBJECTDIR}/_ext/118733356/main.o "/Users/king/MPLABXProjects/Event Service.X/main.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o: /Users/king/MPLABXProjects/Event\ Service.X/NavigateToISZSubHSM.c  .generated_files/flags/default/fcfc232e1ab950884b4834647c914afbf45219cc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o.d" -o ${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o "/Users/king/MPLABXProjects/Event Service.X/NavigateToISZSubHSM.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotDebug.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotDebug.c  .generated_files/flags/default/3dd64386b46df06a6701ad517b91373b4f90b81d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotDebug.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotDebug.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotDebug.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotDebug.o "/Users/king/MPLABXProjects/Event Service.X/RobotDebug.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotEventCheckers.c  .generated_files/flags/default/9c00c46cdde81229d1f2f4383efd66059d9e199b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o "/Users/king/MPLABXProjects/Event Service.X/RobotEventCheckers.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotHardware.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotHardware.c  .generated_files/flags/default/95ba636a1217939503cc698ddb2cd671f7a61e8b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotHardware.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotHardware.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotHardware.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotHardware.o "/Users/king/MPLABXProjects/Event Service.X/RobotHardware.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotHSM.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotHSM.c  .generated_files/flags/default/6f81149d853c8d9482ae0b44e8d4b850131bbdb1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotHSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotHSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotHSM.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotHSM.o "/Users/king/MPLABXProjects/Event Service.X/RobotHSM.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotIMU.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotIMU.c  .generated_files/flags/default/3c3a52314edb224a9b61d66907728d2903a1b04b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotIMU.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotIMU.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotIMU.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotIMU.o "/Users/king/MPLABXProjects/Event Service.X/RobotIMU.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotLauncher.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotLauncher.c  .generated_files/flags/default/67d9bd75c66a5e5bd03b74fedb09ccdf66f6d014 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotLauncher.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotLauncher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotLauncher.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotLauncher.o "/Users/king/MPLABXProjects/Event Service.X/RobotLauncher.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotMotion.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotMotion.c  .generated_files/flags/default/87afca76fe785c49828575fda3e9b26975d961c1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotMotion.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotMotion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotMotion.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotMotion.o "/Users/king/MPLABXProjects/Event Service.X/RobotMotion.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotPlugPlay.c  .generated_files/flags/default/2eaeb83a99d4046cf895ebe3da6eb45c2122f6bf .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o "/Users/king/MPLABXProjects/Event Service.X/RobotPlugPlay.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotSensors.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotSensors.c  .generated_files/flags/default/142edb7b9291c0754485ec176b60c49ef72320b3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotSensors.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotSensors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotSensors.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotSensors.o "/Users/king/MPLABXProjects/Event Service.X/RobotSensors.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotStepper.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotStepper.c  .generated_files/flags/default/db412913bc795621796221851eac779f2d250956 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotStepper.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotStepper.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotStepper.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotStepper.o "/Users/king/MPLABXProjects/Event Service.X/RobotStepper.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotTestHarness.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotTestHarness.c  .generated_files/flags/default/57c3d6269db6fc2ac046caabbddbe9438ddce7d6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotTestHarness.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotTestHarness.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotTestHarness.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotTestHarness.o "/Users/king/MPLABXProjects/Event Service.X/RobotTestHarness.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o: /Users/king/MPLABXProjects/Event\ Service.X/ShootingSubHSM.c  .generated_files/flags/default/9aef5883bb349e4ce947d015d3c94243475c8b94 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o.d" -o ${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o "/Users/king/MPLABXProjects/Event Service.X/ShootingSubHSM.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/830340023/AD.o: /Users/king/VSCode/ECE118/src/AD.c  .generated_files/flags/default/1bf5b785e4dc74460f0573cb1b21134f496e8bd3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/AD.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/AD.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/AD.o.d" -o ${OBJECTDIR}/_ext/830340023/AD.o /Users/king/VSCode/ECE118/src/AD.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/BOARD.o: /Users/king/VSCode/ECE118/src/BOARD.c  .generated_files/flags/default/1198ce797fcf69694f97ee9fef787c0b756bd9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/BOARD.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/BOARD.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/BOARD.o.d" -o ${OBJECTDIR}/_ext/830340023/BOARD.o /Users/king/VSCode/ECE118/src/BOARD.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o: /Users/king/VSCode/ECE118/src/ES_CheckEvents.c  .generated_files/flags/default/e85eec0ff37a82d5f192b2e388ec7c1a203df856 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_CheckEvents.o /Users/king/VSCode/ECE118/src/ES_CheckEvents.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_Framework.o: /Users/king/VSCode/ECE118/src/ES_Framework.c  .generated_files/flags/default/237ce1b9c09374dc8c5d0449c9da492d0f9cbb5f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Framework.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Framework.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_Framework.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_Framework.o /Users/king/VSCode/ECE118/src/ES_Framework.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o: /Users/king/VSCode/ECE118/src/ES_KeyboardInput.c  .generated_files/flags/default/7d6807e4f05d8135cc9e4b41218237940747703b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_KeyboardInput.o /Users/king/VSCode/ECE118/src/ES_KeyboardInput.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_PostList.o: /Users/king/VSCode/ECE118/src/ES_PostList.c  .generated_files/flags/default/df2d2b673073b0afe08d3dc74ccf18a1f713cb18 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_PostList.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_PostList.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_PostList.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_PostList.o /Users/king/VSCode/ECE118/src/ES_PostList.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_Queue.o: /Users/king/VSCode/ECE118/src/ES_Queue.c  .generated_files/flags/default/f34bada9cf674d4863bcc4d5f7d933980ebd26ca .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_Queue.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_Queue.o /Users/king/VSCode/ECE118/src/ES_Queue.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_TattleTale.o: /Users/king/VSCode/ECE118/src/ES_TattleTale.c  .generated_files/flags/default/e9f2c8fb235c5268ccb2bb289354c46b12cc0cd0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_TattleTale.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_TattleTale.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_TattleTale.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_TattleTale.o /Users/king/VSCode/ECE118/src/ES_TattleTale.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/ES_Timers.o: /Users/king/VSCode/ECE118/src/ES_Timers.c  .generated_files/flags/default/de4c7c18b1011fdf62f09e340b0f3d9e5415d91a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/ES_Timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/ES_Timers.o.d" -o ${OBJECTDIR}/_ext/830340023/ES_Timers.o /Users/king/VSCode/ECE118/src/ES_Timers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/IO_Ports.o: /Users/king/VSCode/ECE118/src/IO_Ports.c  .generated_files/flags/default/54f4494acc5bbf4ac81e9aa8b7262dae8794ee8a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/IO_Ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/IO_Ports.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/IO_Ports.o.d" -o ${OBJECTDIR}/_ext/830340023/IO_Ports.o /Users/king/VSCode/ECE118/src/IO_Ports.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/LED.o: /Users/king/VSCode/ECE118/src/LED.c  .generated_files/flags/default/57eee0df1f24d59e461fc0ddc986feebc5aab030 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/LED.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/LED.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/LED.o.d" -o ${OBJECTDIR}/_ext/830340023/LED.o /Users/king/VSCode/ECE118/src/LED.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/pwm.o: /Users/king/VSCode/ECE118/src/pwm.c  .generated_files/flags/default/fa31ca0aec8a5ae11d7622a763ab1339fb3cf649 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/pwm.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/pwm.o.d" -o ${OBJECTDIR}/_ext/830340023/pwm.o /Users/king/VSCode/ECE118/src/pwm.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/RC_Servo.o: /Users/king/VSCode/ECE118/src/RC_Servo.c  .generated_files/flags/default/2502f2e637642f7214ec09a4718951ccc8147944 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/RC_Servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/RC_Servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/RC_Servo.o.d" -o ${OBJECTDIR}/_ext/830340023/RC_Servo.o /Users/king/VSCode/ECE118/src/RC_Servo.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/roach.o: /Users/king/VSCode/ECE118/src/roach.c  .generated_files/flags/default/5ea151c2d642982404002b3bb95371bd0dec43ae .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/roach.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/roach.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/roach.o.d" -o ${OBJECTDIR}/_ext/830340023/roach.o /Users/king/VSCode/ECE118/src/roach.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/serial.o: /Users/king/VSCode/ECE118/src/serial.c  .generated_files/flags/default/6494e788f358c6443debaec65b2c43f8471a1729 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/serial.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/serial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/serial.o.d" -o ${OBJECTDIR}/_ext/830340023/serial.o /Users/king/VSCode/ECE118/src/serial.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/830340023/timers.o: /Users/king/VSCode/ECE118/src/timers.c  .generated_files/flags/default/c5fcce8d7693d3fb7b4ddebd9e7f6de38fdf58e1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/830340023" 
	@${RM} ${OBJECTDIR}/_ext/830340023/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/830340023/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/830340023/timers.o.d" -o ${OBJECTDIR}/_ext/830340023/timers.o /Users/king/VSCode/ECE118/src/timers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/AlignSubHSM.o: /Users/king/MPLABXProjects/Event\ Service.X/AlignSubHSM.c  .generated_files/flags/default/3a0edf3d8076b3331540a4db5cef21cef6b3e4ec .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/AlignSubHSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/AlignSubHSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/AlignSubHSM.o.d" -o ${OBJECTDIR}/_ext/118733356/AlignSubHSM.o "/Users/king/MPLABXProjects/Event Service.X/AlignSubHSM.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o: /Users/king/MPLABXProjects/Event\ Service.X/FindFrontTapeSubHSM.c  .generated_files/flags/default/f53177736784547a76da2fb05d604985e7631c32 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o.d" -o ${OBJECTDIR}/_ext/118733356/FindFrontTapeSubHSM.o "/Users/king/MPLABXProjects/Event Service.X/FindFrontTapeSubHSM.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/main.o: /Users/king/MPLABXProjects/Event\ Service.X/main.c  .generated_files/flags/default/be3c77d2c989f09c3c9101eb0ca19641b7dd368d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/main.o.d" -o ${OBJECTDIR}/_ext/118733356/main.o "/Users/king/MPLABXProjects/Event Service.X/main.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o: /Users/king/MPLABXProjects/Event\ Service.X/NavigateToISZSubHSM.c  .generated_files/flags/default/8b39f90a9ed665e1496b54f7f83a3330d912e785 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o.d" -o ${OBJECTDIR}/_ext/118733356/NavigateToISZSubHSM.o "/Users/king/MPLABXProjects/Event Service.X/NavigateToISZSubHSM.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotDebug.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotDebug.c  .generated_files/flags/default/ae190a919371c9d39f52609a939cc365f3139e8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotDebug.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotDebug.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotDebug.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotDebug.o "/Users/king/MPLABXProjects/Event Service.X/RobotDebug.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotEventCheckers.c  .generated_files/flags/default/b37980cabc1d77b8cca01e1837e13c004d5ca6e2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotEventCheckers.o "/Users/king/MPLABXProjects/Event Service.X/RobotEventCheckers.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotHardware.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotHardware.c  .generated_files/flags/default/c20a982b87eadd886c262cd5f9a0ad3b579bdfc1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotHardware.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotHardware.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotHardware.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotHardware.o "/Users/king/MPLABXProjects/Event Service.X/RobotHardware.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotHSM.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotHSM.c  .generated_files/flags/default/ab608d8c67147a340d0711878aee0216be4d9450 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotHSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotHSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotHSM.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotHSM.o "/Users/king/MPLABXProjects/Event Service.X/RobotHSM.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotIMU.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotIMU.c  .generated_files/flags/default/365c16c0e030e837e65cbaefb3b141423e2f45f9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotIMU.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotIMU.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotIMU.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotIMU.o "/Users/king/MPLABXProjects/Event Service.X/RobotIMU.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotLauncher.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotLauncher.c  .generated_files/flags/default/bbc16523c5b2d59f105579866ca9bd7f99f980ea .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotLauncher.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotLauncher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotLauncher.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotLauncher.o "/Users/king/MPLABXProjects/Event Service.X/RobotLauncher.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotMotion.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotMotion.c  .generated_files/flags/default/79cacd8e227c5f5db61755908b6abd109aa73f7c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotMotion.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotMotion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotMotion.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotMotion.o "/Users/king/MPLABXProjects/Event Service.X/RobotMotion.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotPlugPlay.c  .generated_files/flags/default/5e7749497faff12a2d818bd5eb72142320c6a453 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotPlugPlay.o "/Users/king/MPLABXProjects/Event Service.X/RobotPlugPlay.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotSensors.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotSensors.c  .generated_files/flags/default/51857d62b1c58d26e94f708430cac7c15848264f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotSensors.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotSensors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotSensors.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotSensors.o "/Users/king/MPLABXProjects/Event Service.X/RobotSensors.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotStepper.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotStepper.c  .generated_files/flags/default/1b45bc0ed79ffb38838827385b68de95321d7be5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotStepper.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotStepper.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotStepper.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotStepper.o "/Users/king/MPLABXProjects/Event Service.X/RobotStepper.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/RobotTestHarness.o: /Users/king/MPLABXProjects/Event\ Service.X/RobotTestHarness.c  .generated_files/flags/default/72663560e3c09b7b3199288b6e17ae148b103d50 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotTestHarness.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/RobotTestHarness.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/RobotTestHarness.o.d" -o ${OBJECTDIR}/_ext/118733356/RobotTestHarness.o "/Users/king/MPLABXProjects/Event Service.X/RobotTestHarness.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o: /Users/king/MPLABXProjects/Event\ Service.X/ShootingSubHSM.c  .generated_files/flags/default/336aeac027c4aab999de27e98650075c67d5ae21 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/118733356" 
	@${RM} ${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O0 -fno-common -I"../../VSCode/ECE118/include" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o.d" -o ${OBJECTDIR}/_ext/118733356/ShootingSubHSM.o "/Users/king/MPLABXProjects/Event Service.X/ShootingSubHSM.c"    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/Event_Service.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    /Users/king/VSCode/ECE118/bootloader320.ld
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_SIMULATOR=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o ${DISTDIR}/Event_Service.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)      -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,${DISTDIR}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
${DISTDIR}/Event_Service.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   /Users/king/VSCode/ECE118/bootloader320.ld
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o ${DISTDIR}/Event_Service.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,${DISTDIR}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}/xc32-bin2hex ${DISTDIR}/Event_Service.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
