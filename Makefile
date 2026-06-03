#
#  There exist several targets which are by default empty and which can be 
#  used for execution of your targets. These targets are usually executed 
#  before and after some main targets. They are: 
#
#     .build-pre:              called before 'build' target
#     .build-post:             called after 'build' target
#     .clean-pre:              called before 'clean' target
#     .clean-post:             called after 'clean' target
#     .clobber-pre:            called before 'clobber' target
#     .clobber-post:           called after 'clobber' target
#     .all-pre:                called before 'all' target
#     .all-post:               called after 'all' target
#     .help-pre:               called before 'help' target
#     .help-post:              called after 'help' target
#
#  Targets beginning with '.' are not intended to be called on their own.
#
#  Main targets can be executed directly, and they are:
#  
#     build                    build a specific configuration
#     clean                    remove built files from a configuration
#     clobber                  remove all built files
#     all                      build all configurations
#     help                     print help mesage
#  
#  Targets .build-impl, .clean-impl, .clobber-impl, .all-impl, and
#  .help-impl are implemented in nbproject/makefile-impl.mk.
#
#  Available make variables:
#
#     CND_BASEDIR                base directory for relative paths
#     CND_DISTDIR                default top distribution directory (build artifacts)
#     CND_BUILDDIR               default top build directory (object files, ...)
#     CONF                       name of current configuration
#     CND_ARTIFACT_DIR_${CONF}   directory of build artifact (current configuration)
#     CND_ARTIFACT_NAME_${CONF}  name of build artifact (current configuration)
#     CND_ARTIFACT_PATH_${CONF}  path to build artifact (current configuration)
#     CND_PACKAGE_DIR_${CONF}    directory of package (current configuration)
#     CND_PACKAGE_NAME_${CONF}   name of package (current configuration)
#     CND_PACKAGE_PATH_${CONF}   path to package (current configuration)
#
# NOCDDL


# Environment 
MKDIR=mkdir
CP=cp
CCADMIN=CCadmin
RANLIB=ranlib


# Local project overrides:
# -I"." lets the shared framework sources find this project's ES_Configure.h
MP_EXTRA_CC_PRE += -I"."

# Command-line helpers:
#   make ROBOT_DEBUG=1    -> enable state-entry logs
#   make ROBOT_TEST=1     -> enable keyboard event injection and state logs
#   make MOTOR_SENSOR_TEST=1 -> direct motor/sensor bench harness, no HSM
#   make MOTOR_SENSOR_TEST=1 IMU_ALIGN_BENCH=1 -> add gyro-zero/jitter-align bench keys
#   make BEACON_TEST=1 -> direct beacon ADC smoothing harness, no HSM
#   make GPIO_HIGH_TEST=1 -> set connector GPIO pins high for probing
#   make ROBOT_TRACE=1    -> enable verbose framework Run... trace output
#   Add plug-play hardware in test mode with HW_IMU=1, HW_MOTORS=1,
#   HW_BEACON=1, HW_TAPE=1, HW_BUMP=1, HW_SERVO=1,
#   HW_SHOOTER=1, HW_SHOOTER_ADC=1, or HW_STEPPER=1.
ifdef ROBOT_DEBUG
MP_EXTRA_CC_PRE += -DROBOT_DEBUG
endif

ifdef ROBOT_TEST
MP_EXTRA_CC_PRE += -DROBOT_KEYBOARD_TEST -DROBOT_DEBUG
endif

ifdef MOTOR_SENSOR_TEST
MP_EXTRA_CC_PRE += -DROBOT_MOTOR_SENSOR_TEST -DROBOT_DEBUG \
    -DROBOT_HW_USE_DRIVE_MOTORS -DROBOT_HW_USE_TAPE \
    -DROBOT_HW_USE_BUMP -DROBOT_HW_USE_BEACON_ADC \
    -DROBOT_HW_USE_SHOOTER_MOTOR -DROBOT_HW_USE_STEPPER
endif

ifdef IMU_ALIGN_BENCH
MP_EXTRA_CC_PRE += -DROBOT_IMU_ALIGN_BENCH
endif

ifdef BEACON_TEST
MP_EXTRA_CC_PRE += -DROBOT_BEACON_TEST -DROBOT_DEBUG \
    -DROBOT_HW_USE_BEACON_ADC
endif

ifdef GPIO_HIGH_TEST
MP_EXTRA_CC_PRE += -DROBOT_GPIO_HIGH_TEST -DROBOT_DEBUG
endif

ifdef ROBOT_TRACE
MP_EXTRA_CC_PRE += -DROBOT_TRACE
endif

ifdef HW_IMU
MP_EXTRA_CC_PRE += -DROBOT_HW_USE_BNO055
endif

ifdef HW_MOTORS
MP_EXTRA_CC_PRE += -DROBOT_HW_USE_DRIVE_MOTORS
endif

ifdef HW_BEACON
MP_EXTRA_CC_PRE += -DROBOT_HW_USE_BEACON_ADC
endif

ifdef HW_TAPE
MP_EXTRA_CC_PRE += -DROBOT_HW_USE_TAPE
endif

# Solenoid hardware placement is intentionally disabled for now.
# ifdef HW_SOLENOID
# MP_EXTRA_CC_PRE += -DROBOT_HW_USE_SOLENOID_ADC
# endif

ifdef HW_BUMP
MP_EXTRA_CC_PRE += -DROBOT_HW_USE_BUMP
endif

ifdef HW_SERVO
MP_EXTRA_CC_PRE += -DROBOT_HW_USE_LAUNCHER_SERVO
endif

ifdef HW_SHOOTER
MP_EXTRA_CC_PRE += -DROBOT_HW_USE_SHOOTER_MOTOR
endif

ifdef HW_SHOOTER_ADC
MP_EXTRA_CC_PRE += -DROBOT_HW_USE_SHOOTER_ADC
endif

ifdef HW_STEPPER
MP_EXTRA_CC_PRE += -DROBOT_HW_USE_STEPPER
endif

ifdef HW_EXTRA_DEFINES
MP_EXTRA_CC_PRE += $(foreach def,$(HW_EXTRA_DEFINES),-D$(def))
endif


# build
build: .build-post

.build-pre:
# Add your pre 'build' code here...

.build-post: .build-impl
# Add your post 'build' code here...


# clean
clean: .clean-post

.clean-pre:
# Add your pre 'clean' code here...
# WARNING: the IDE does not call this target since it takes a long time to
# simply run make. Instead, the IDE removes the configuration directories
# under build and dist directly without calling make.
# This target is left here so people can do a clean when running a clean
# outside the IDE.

.clean-post: .clean-impl
# Add your post 'clean' code here...


# clobber
clobber: .clobber-post

.clobber-pre:
# Add your pre 'clobber' code here...

.clobber-post: .clobber-impl
# Add your post 'clobber' code here...


# all
all: .all-post

.all-pre:
# Add your pre 'all' code here...

.all-post: .all-impl
# Add your post 'all' code here...


# help
help: .help-post

.help-pre:
# Add your pre 'help' code here...

.help-post: .help-impl
# Add your post 'help' code here...



# include project implementation makefile
include nbproject/Makefile-impl.mk

# include project make variables
include nbproject/Makefile-variables.mk
