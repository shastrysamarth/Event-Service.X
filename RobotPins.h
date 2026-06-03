#ifndef ROBOT_PINS_H
#define ROBOT_PINS_H

#include "AD.h"
#include "IO_Ports.h"
#include "RC_Servo.h"
#include "pwm.h"


#define CHASSIS_WIDTH_IN 10.0f
#define CHASSIS_LENGTH_IN 10.0f
#define ROBOT_HALF_WIDTH_IN (CHASSIS_WIDTH_IN / 2.0f)
#define ROBOT_HALF_LENGTH_IN (CHASSIS_LENGTH_IN / 2.0f)

#define MOTOR_SPEED_IPS 4.0f
#define MOTOR_BENCH_SPEED_IPS 4.0f
#define STRAFE_SPEED_IPS 5.0f
#define ALIGN_SPEED_IPS 4.0f
#define TURN_SPEED_IPS STRAFE_SPEED_IPS
#define FIND_FRONT_TURN_SPEED_IPS MOTOR_SPEED_IPS
#define MOTOR_DUTY_PER_IPS 70.0f
#define MOTOR_MIN_ACTIVE_DUTY 0u

#define FIND_FRONT_IMU_SETTLE_MS 3000u
#define TURN_IMU_SETTLE_MS 100u
#define SHOOT_TIME_MS 120000u

/* FindFrontTape approach timings (reuse FIND_FRONT_IMU_TIMER, states are serial). */
#define FRONT_TAPE_CONFIRM_MS 20u
#define FRONT_TAPE_BACKUP_MS 500u
#define FRONT_TAPE_RECOVER_TURN_MS 2u

#define FIND_FRONT_IMU_TIMER 0
#define NAV_SETTLE_TIMER 1
#define SHOOT_TIMER 2

/* DEPRECATED with heading-only align: was used for IMU position error vs ref. */
#define POSITION_THRESHOLD_IN 0.75f
#define HEADING_THRESHOLD_DEG 3.0f
#define TAPE_ALIGN_HEADING_STRAIGHT_DEG 1.5f
/* Gyro align (granular pulse/brake): turn pulse length scales with |heading
 * error| between MIN and MAX so small corrections stay fine-grained but the
 * motors still break static friction (2 ms was too short and never moved).
 * BRAKE_MS should be >= ~2x ROBOT_IMU_UPDATE_PERIOD_MS so the heading sample
 * after each pulse is fresh. MAX_PULSES is a watchdog if the robot cannot
 * converge. */
#define GYRO_ALIGN_STRAIGHT_DEG 1.25f
#define GYRO_ALIGN_TURN_PULSE_MIN_MS 28u
#define GYRO_ALIGN_TURN_PULSE_MAX_MS 55u
#define GYRO_ALIGN_BRAKE_MS 50u
#define GYRO_ALIGN_MAX_PULSES 40u
/* DEPRECATED name: use GYRO_ALIGN_TURN_PULSE_MIN_MS / _MAX_MS instead. */
#define GYRO_ALIGN_TURN_PULSE_MS GYRO_ALIGN_TURN_PULSE_MIN_MS
#define TAPE_ALIGN_SWEEP_TIMER_MS 300u
/* Tape align heading-correction turn slice: motors stall below ~4 IPS, so the
 * heading is squared in short pulse/rest cycles instead of a continuous turn. */
#define ALIGN_MOTION_PULSE_MS 100u
/* Tape align search sweep: first run is START ms, then each direction reversal
 * grows the run by STEP ms (widening back/forth search for the tape-5 edge). */
#define ALIGN_TIMER_START_MS 100u
#define ALIGN_TIMER_STEP_MS 100u
/* Small deadband on accel used for integration (LSB ≈ 0.01 m/s²). */
#define IMU_ACCEL_INTEGRATE_DEADBAND_RAW 8
/* "Still" uses raw linear accel before integrate deadband (same LSB scale). */
#define IMU_STILL_ACCEL_RAW 45
#define IMU_STILL_GYRO_RAW 12
#define IMU_STATIONARY_CONFIRM_MS 400u
/* DEPRECATED: heading rotation sign for field-frame accel integration (cosf/sinf removed). */
/* #define IMU_HEADING_ROT_SIGN (-1.0f) */
#define ROBOT_IMU_UPDATE_PERIOD_MS 20u
#define IMU_MODE_CHECK_PERIOD_MS 250u
#define IMU_DEBUG_STREAM_PERIOD_MS 250u
#define ALIGN_STABLE_SAMPLE_COUNT 3u
#ifndef ROBOT_REALTIME_TRACE
#define ROBOT_REALTIME_TRACE 0
#endif
/* Master switch for the high-rate per-event trace logs. These print on every
 * event and can back up the UART / event queues and slow sensor sampling. Set
 * to 0 to silence them (one-shot init + snapshot logs still obey ROBOT_DEBUG);
 * set to 1 to restore the verbose tracing while debugging.
 *
 * Each log family below defaults to this global value but can be overridden
 * individually on the build command line, e.g.:
 *     -DROBOT_CHATTY_LOGS=0 -DROBOT_LOG_MOTOR=1 -DROBOT_LOG_ALIGN=1
 * turns everything off except the motor-control and align traces. */
#ifndef ROBOT_CHATTY_LOGS
#define ROBOT_CHATTY_LOGS 1
#endif
/* [STATE] state-entry logs (RobotDebug_LogStateEntry / ROBOT_DEBUG_STATE). */
#ifndef ROBOT_LOG_STATE
#define ROBOT_LOG_STATE ROBOT_CHATTY_LOGS
#endif
/* [MOTOR] drive command + "(stop by <fn>)" control-change logs. */
#ifndef ROBOT_LOG_MOTOR
#define ROBOT_LOG_MOTOR ROBOT_CHATTY_LOGS
#endif
/* [TAPE] change-mask logs from the tape event checker. */
#ifndef ROBOT_LOG_TAPE
#define ROBOT_LOG_TAPE ROBOT_CHATTY_LOGS
#endif
/* [BUMP] change-mask logs from the bump event checker. */
#ifndef ROBOT_LOG_BUMP
#define ROBOT_LOG_BUMP ROBOT_CHATTY_LOGS
#endif
/* [NAV] per-event + align-start/done traces in NavigateToISZ. */
#ifndef ROBOT_LOG_NAV
#define ROBOT_LOG_NAV ROBOT_CHATTY_LOGS
#endif
/* [ALIGN] gyro heading + turn-direction traces while aligning. */
#ifndef ROBOT_LOG_ALIGN
#define ROBOT_LOG_ALIGN ROBOT_CHATTY_LOGS
#endif
/* [IMU] heading re-zero events (RobotIMU_ZeroHeading). */
#ifndef ROBOT_LOG_IMU
#define ROBOT_LOG_IMU ROBOT_CHATTY_LOGS
#endif
/* RealignedEvent EventParam: sensor path runs IMU zero + ref re-anchor in Navigate. */
#define ALIGN_REALIGNED_SOURCE_MANUAL (0u)
#define ALIGN_REALIGNED_SOURCE_SENSOR (1u)
/* DEPRECATED with the gyro-only NavigateToISZ rework: the old corner-approach
 * distance moves are no longer used by NavigateToISZSubHSM. */
#define DISTANCE_FORWARD_TO_ISZ_IN 10.0f
#define DISTANCE_REVERSE_TO_SHOOT_IN 3.0f

/* NavigateToISZ (gyro-only rework) timings.
 *   NAV_TAPE5_WAIT_MS    : after a corner tape drops, wait this long for the
 *                          center tape (sensor 5) before sweeping the far way.
 *   NAV_FORWARD_AFTER_BUMP_MS : short forward nudge off a bumped wall.
 *   NAV_FORWARD_TO_ISZ_MS     : forward creep before the final ISZ strafe.
 *   NAV_FINAL_STRAFE_IN       : open-loop strafe distance into the ISZ. */
#define NAV_TAPE5_WAIT_MS 800u
#define NAV_FORWARD_AFTER_BUMP_MS 100u
#define NAV_FORWARD_TO_ISZ_MS 200u
#define NAV_FINAL_STRAFE_IN 5.0f

/* Tape event params use bit = 1 when that tape sensor is on tape. */
#define TAPE_SENSOR_1_MASK (1u << 0)
#define TAPE_SENSOR_2_MASK (1u << 1)
#define TAPE_SENSOR_3_MASK (1u << 2)
#define TAPE_SENSOR_4_MASK (1u << 3)
#define TAPE_SENSOR_5_MASK (1u << 4)
#define TAPE_SENSOR_ALL_MASK (TAPE_SENSOR_1_MASK | TAPE_SENSOR_2_MASK | \
    TAPE_SENSOR_3_MASK | TAPE_SENSOR_4_MASK | TAPE_SENSOR_5_MASK)
#define TAPE_EVENT_CHANGED_SHIFT 8u
#define TAPE_EVENT_MAKE_PARAM(currentMask, changedMask) \
    ((uint16_t) (((uint16_t) ((changedMask) & TAPE_SENSOR_ALL_MASK) << \
    TAPE_EVENT_CHANGED_SHIFT) | ((currentMask) & TAPE_SENSOR_ALL_MASK)))
#define TAPE_EVENT_CURRENT_MASK(eventParam) \
    ((uint8_t) ((eventParam) & TAPE_SENSOR_ALL_MASK))
#define TAPE_EVENT_CHANGED_MASK(eventParam) \
    ((uint8_t) (((eventParam) >> TAPE_EVENT_CHANGED_SHIFT) & \
    TAPE_SENSOR_ALL_MASK))

/* Bump event params use bit = 1 when that bump sensor is pressed.
 * Bump sensors: bit0 = FR (1), bit1 = FL (2), bit2 = RR (3), bit3 = RL (4). */
#define BUMP_SENSOR_1_MASK (1u << 0)
#define BUMP_SENSOR_2_MASK (1u << 1)
#define BUMP_SENSOR_3_MASK (1u << 2)
#define BUMP_SENSOR_4_MASK (1u << 3)
#define BUMP_SENSOR_ALL_MASK (BUMP_SENSOR_1_MASK | BUMP_SENSOR_2_MASK | \
    BUMP_SENSOR_3_MASK | BUMP_SENSOR_4_MASK)
#define BUMP_EVENT_CHANGED_SHIFT 8u
#define BUMP_EVENT_MAKE_PARAM(currentMask, changedMask) \
    ((uint16_t) (((uint16_t) ((changedMask) & BUMP_SENSOR_ALL_MASK) << \
    BUMP_EVENT_CHANGED_SHIFT) | ((currentMask) & BUMP_SENSOR_ALL_MASK)))
#define BUMP_EVENT_CURRENT_MASK(eventParam) \
    ((uint8_t) ((eventParam) & BUMP_SENSOR_ALL_MASK))
#define BUMP_EVENT_CHANGED_MASK(eventParam) \
    ((uint8_t) (((eventParam) >> BUMP_EVENT_CHANGED_SHIFT) & \
    BUMP_SENSOR_ALL_MASK))

#define BNO055_AXIS_X 0u
#define BNO055_AXIS_Y 1u
#define BNO055_AXIS_Z 2u
#define BNO055_EULER_HEADING 0u
#define BNO055_EULER_ROLL 1u
#define BNO055_EULER_PITCH 2u
/* BNO055 mount:
 * Starting from flat/facing forward, the board is rotated +90 deg about sensor X
 * so it is vertical, then yawed right 90 deg about the chassis vertical axis.
 * BNO055 Euler output is heading/roll/pitch, not sensor X/Y/Z, so robot yaw
 * must use Euler heading. Sensor-axis mapping below is only for gyro/accel.
 */
#define BNO055_HEADING_EULER_INDEX BNO055_EULER_HEADING
#define BNO055_HEADING_EULER_SIGN 1
#define BNO055_HEADING_GYRO_AXIS BNO055_AXIS_Y
#define BNO055_HEADING_GYRO_SIGN 1
#define BNO055_ROBOT_X_ACCEL_AXIS BNO055_AXIS_Z
#define BNO055_ROBOT_X_ACCEL_SIGN (-1)
#define BNO055_ROBOT_Y_ACCEL_AXIS BNO055_AXIS_X
#define BNO055_ROBOT_Y_ACCEL_SIGN (-1)

/* HW-201 tape sensors: OUT is HIGH on tape and LOW off tape. */
#define TAPE_SENSOR_1_PORT PORTZ
#define TAPE_SENSOR_1_PIN  PIN4
#define TAPE_SENSOR_2_PORT PORTZ
#define TAPE_SENSOR_2_PIN  PIN5
#define TAPE_SENSOR_3_PORT PORTW
#define TAPE_SENSOR_3_PIN  PIN7
#define TAPE_SENSOR_4_PORT PORTZ
#define TAPE_SENSOR_4_PIN  PIN8
#define TAPE_SENSOR_5_PORT PORTZ
#define TAPE_SENSOR_5_PIN  PIN9
#define TAPE_SENSOR_PORTZ_PINS (PIN4 | PIN5 | PIN8 | PIN9)
#define TAPE_SENSOR_PORTW_PINS PIN7
/* Active level: 1 = pin HIGH when on tape, 0 = pin LOW when on tape. */
#define TAPE_SENSOR_1_ON_IS_HIGH 1
#define TAPE_SENSOR_2_ON_IS_HIGH 1
#define TAPE_SENSOR_3_ON_IS_HIGH 1
#define TAPE_SENSOR_4_ON_IS_HIGH 1
#define TAPE_SENSOR_5_ON_IS_HIGH 1
#define TAPE_BLACK_IS_HIGH TAPE_SENSOR_1_ON_IS_HIGH
#define BEACON_ADC_PIN AD_PORTV8

/* Beacon detector peak output is 1.65V-3.0V into the Uno32 ADC.
 * 1.65V sits near ADC 512 past 16 ft, and 3.0V sits near ADC 930 below 6 ft.
 * [FLAG][#1] Replace the linear placeholder with measured calibration data
 * after testing the detector at 6 ft through 16 ft. */
#define BEACON_AVERAGE_SAMPLE_COUNT 50u
#define BEACON_DISTANCE_UNKNOWN_FT 0u
#define BEACON_DISTANCE_MIN_FT 6u
#define BEACON_DISTANCE_MAX_FT 16u
#define BEACON_ADC_AT_6FT 930u
#define BEACON_ADC_AT_16FT 512u

/* Solenoid ADC placements intentionally disabled for now.
 * #define SOLENOID_SENSOR_1_ADC_PIN AD_PORTV3
 * #define SOLENOID_SENSOR_2_ADC_PIN AD_PORTV4
 * #define SOLENOID_SENSOR_3_ADC_PIN AD_PORTV5
 * #define SOLENOID_SENSOR_4_ADC_PIN AD_PORTV6
 * #define SOLENOID_SENSOR_5_ADC_PIN AD_PORTV7
 * #define SOLENOID_SENSOR_6_ADC_PIN AD_PORTV8
 */

/* Bump sensors: digital inputs, active-high, 3.3V when bumped.
 * BUMP_SENSOR_1=FR, 2=FL, 3=RR, 4=RL. */
#define BUMP_SENSOR_1_PORT PORTV
#define BUMP_SENSOR_1_PIN  PIN7
#define BUMP_SENSOR_2_PORT PORTZ
#define BUMP_SENSOR_2_PIN  PIN11
#define BUMP_SENSOR_3_PORT PORTV
#define BUMP_SENSOR_3_PIN  PIN5
#define BUMP_SENSOR_4_PORT PORTV
#define BUMP_SENSOR_4_PIN  PIN6
#define BUMP_SENSOR_PORTV_PINS (PIN5 | PIN6 | PIN7)
#define BUMP_SENSOR_PORTZ_PINS PIN11
/* Active level: 1 = pin HIGH when bump pressed, 0 = pin LOW when bump pressed. */
#define BUMP_ON_IS_HIGH 1

/* [FLAG][#1] Confirm if this is really an ADC command input; W5 now also maps
 * to H-bridge 2 RR IN2, so remap before enabling with drive motors. */
#define SHOOTER_MOTOR_ADC_PIN AD_PORTW5

/* Solenoid ADC group intentionally disabled for now.
 * #define SOLENOID_SENSOR_ADC_PINS (...)
 */

#define ALL_ROBOT_ADC_PINS (BEACON_ADC_PIN | SHOOTER_MOTOR_ADC_PIN)

/* Solenoid threshold intentionally disabled for now.
 * #define SOLENOID_ON_ADC_THRESHOLD 700u
 * #define SOLENOID_ON_IS_HIGH 1
 */
#define BEACON_INCREASE_ADC_DELTA 15u
#define BEACON_DECREASE_ADC_DELTA 8u

/*
 * L298N H-bridge 1 controls front motors:
 *   Channel A / ENA -> motor FL
 *   Channel B / ENB -> motor FR
 * L298N H-bridge 2 controls rear motors:
 *   Channel A / ENA -> motor RR
 *   Channel B / ENB -> motor RL
 */
#define MOTOR_FL_PWM_PIN PWM_PORTZ06
#define MOTOR_FR_PWM_PIN PWM_PORTY12
#define MOTOR_RR_PWM_PIN PWM_PORTY10
#define MOTOR_RL_PWM_PIN PWM_PORTY04
#define SHOOTER_MOTOR_PWM_PIN PWM_PORTX11
#define DRIVE_MOTOR_PWM_PINS (MOTOR_FL_PWM_PIN | MOTOR_FR_PWM_PIN | \
    MOTOR_RL_PWM_PIN | MOTOR_RR_PWM_PIN)
#define ALL_PWM_PINS (MOTOR_FL_PWM_PIN | MOTOR_FR_PWM_PIN | MOTOR_RL_PWM_PIN | \
    MOTOR_RR_PWM_PIN | SHOOTER_MOTOR_PWM_PIN)

/* H-bridge 1 channel A: FL, ENA Z6, IN1 Y8, IN2 W3. */
#define MOTOR_FL_IN1_TRIS PORTY08_TRIS
#define MOTOR_FL_IN1_LAT PORTY08_LAT
#define MOTOR_FL_IN2_TRIS PORTW03_TRIS
#define MOTOR_FL_IN2_LAT PORTW03_LAT

/* H-bridge 1 channel B: FR, ENB Y12, IN3 Y9, IN4 W4. */
#define MOTOR_FR_IN1_TRIS PORTY09_TRIS
#define MOTOR_FR_IN1_LAT PORTY09_LAT
#define MOTOR_FR_IN2_TRIS PORTW04_TRIS
#define MOTOR_FR_IN2_LAT PORTW04_LAT

/* H-bridge 2 channel A: RR, ENA Y10, IN1 Y11, IN2 W5. */
#define MOTOR_RR_IN1_TRIS PORTY11_TRIS
#define MOTOR_RR_IN1_LAT PORTY11_LAT
#define MOTOR_RR_IN2_TRIS PORTW05_TRIS
#define MOTOR_RR_IN2_LAT PORTW05_LAT

/* H-bridge 2 channel B: RL, ENB Y4, IN3 Z3, IN4 W6. */
#define MOTOR_RL_IN1_TRIS PORTZ03_TRIS
#define MOTOR_RL_IN1_LAT PORTZ03_LAT
#define MOTOR_RL_IN2_TRIS PORTW06_TRIS
#define MOTOR_RL_IN2_LAT PORTW06_LAT

/* DRV8811-like stepper module:
 *   ENABLE is hard-wired to constant 3.3V
 *   STEP -> Y6 digital output
 *   DIR  -> Y7 digital output
 *   OUT1/OUT2 -> NEMA17 coil A+/A-
 *   OUT3/OUT4 -> NEMA17 coil B+/B-
 */
#define STEPPER_STEP_TRIS PORTY06_TRIS
#define STEPPER_STEP_LAT PORTY06_LAT
#define STEPPER_DIR_TRIS PORTY07_TRIS
#define STEPPER_DIR_LAT PORTY07_LAT
#define STEPPER_HAS_ENABLE_PIN 0
#define STEPPER_ENABLE_ACTIVE_LOW 0
#define STEPPER_FULL_STEPS_PER_REV 200u
#define STEPPER_FULL_STEP_DEG 1.8f
#define LAUNCHER_STEPPER_MIN_STEP 0
#define LAUNCHER_STEPPER_MAX_STEP 45
#define LAUNCHER_STEPPER_HOME_STEP 0

/* Polarity is calibrated so positive wheel speed means robot-forward wheel motion. */
#define MOTOR_FL_POLARITY (-1)
#define MOTOR_FR_POLARITY (-1)
#define MOTOR_RL_POLARITY (-1)
#define MOTOR_RR_POLARITY (-1)

#define LAUNCHER_SERVO_PIN RC_PORTZ08
#define LAUNCHER_SERVO_MIN_PULSE_US 1000u
#define LAUNCHER_SERVO_MAX_PULSE_US 2000u
#define LAUNCHER_SERVO_CENTER_PULSE_US 1500u
#define SHOOTER_MOTOR_DUTY MAX_PWM

#define BNO055_I2C_ADDRESS 0x28u
#define BNO055_I2C_BAUD_HZ 100000u
/* Bit-bang I2C half-clock delay: NOP spins per SCL edge in I2CDelay(). Lower =
 * faster bus and shorter blocking per IMU read (BNO055 supports up to 400kHz).
 * Was 220; reduced to speed up I2C. If the IMU stops responding (readOk=0),
 * raise this back toward 220. Overridable from the build command line. */
#ifndef I2C_HALF_CLOCK_NOPS
#define I2C_HALF_CLOCK_NOPS 60u
#endif

/* BNO055 I2C hookup on the Uno32 stack. */
#define BNO055_SDA_PIN_LABEL "V3"
#define BNO055_SCL_PIN_LABEL "V4"
#define BNO055_PIN_LABEL "SDA -> V3 digital I/O, SCL -> V4 digital I/O, Euler heading tracks flat yaw"
#define BEACON_ADC_PIN_LABEL "V8 analog input"
#define TAPE_SENSOR_1_PIN_LABEL "HW-201 OUT -> Z4 digital input, HIGH on tape"
#define TAPE_SENSOR_2_PIN_LABEL "HW-201 OUT -> Z5 digital input, HIGH on tape"
#define TAPE_SENSOR_3_PIN_LABEL "HW-201 OUT -> W7 digital input, HIGH on tape"
#define TAPE_SENSOR_4_PIN_LABEL "HW-201 OUT -> Z8 digital input, HIGH on tape"
#define TAPE_SENSOR_5_PIN_LABEL "HW-201 OUT -> Z9 digital input, HIGH on tape"
/* Solenoid pin labels intentionally disabled for now.
 * #define SOLENOID_SENSOR_1_PIN_LABEL ""
 * #define SOLENOID_SENSOR_2_PIN_LABEL ""
 * #define SOLENOID_SENSOR_3_PIN_LABEL ""
 * #define SOLENOID_SENSOR_4_PIN_LABEL ""
 * #define SOLENOID_SENSOR_5_PIN_LABEL ""
 * #define SOLENOID_SENSOR_6_PIN_LABEL ""
 */
#define BUMP_SENSOR_1_PIN_LABEL "FR bump OUT -> V7 digital input, HIGH when bumped"
#define BUMP_SENSOR_2_PIN_LABEL "FL bump OUT -> Z11 digital input, HIGH when bumped"
#define BUMP_SENSOR_3_PIN_LABEL "RR bump OUT -> V5 digital input, HIGH when bumped"
#define BUMP_SENSOR_4_PIN_LABEL "RL bump OUT -> V6 digital input, HIGH when bumped"
#define MOTOR_FL_PIN_LABEL "H-bridge 1 A: ENA PWM Z6, IN1 Y8, IN2 W3"
#define MOTOR_FR_PIN_LABEL "H-bridge 1 B: ENB PWM Y12, IN3 Y9, IN4 W4"
#define MOTOR_RL_PIN_LABEL "H-bridge 2 B: ENB PWM Y4, IN3 Z3, IN4 W6"
#define MOTOR_RR_PIN_LABEL "H-bridge 2 A: ENA PWM Y10, IN1 Y11, IN2 W5"
#define LAUNCHER_SERVO_PIN_LABEL "RC servo Z8"
#define SHOOTER_MOTOR_PIN_LABEL "TIP122 input -> PWM X11"
#define SHOOTER_MOTOR_ADC_PIN_LABEL "[FLAG][#1] remap; W5 is H-bridge 2 RR IN2"
#define STEPPER_PIN_LABEL "ENABLE -> constant 3.3V, STEP -> Y6, DIR -> Y7"

#endif /* ROBOT_PINS_H */
