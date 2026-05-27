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

#define MOTOR_SPEED_IPS 9.0f
#define ALIGN_SPEED_IPS 3.0f
#define TURN_SPEED_IPS MOTOR_SPEED_IPS
#define MOTOR_DUTY_PER_IPS 70.0f
#define MOTOR_MIN_ACTIVE_DUTY 350u

#define FIND_FRONT_IMU_SETTLE_MS 3000u
#define TURN_IMU_SETTLE_MS 3000u
#define SHOOT_TIME_MS 120000u

#define FIND_FRONT_IMU_TIMER 0
#define NAV_SETTLE_TIMER 1
#define SHOOT_TIMER 2

/* DEPRECATED with heading-only align: was used for IMU position error vs ref. */
#define POSITION_THRESHOLD_IN 0.75f
#define HEADING_THRESHOLD_DEG 3.0f
/* Small deadband on accel used for integration (LSB ≈ 0.01 m/s²). */
#define IMU_ACCEL_INTEGRATE_DEADBAND_RAW 8
/* "Still" uses raw linear accel before integrate deadband (same LSB scale). */
#define IMU_STILL_ACCEL_RAW 45
#define IMU_STILL_GYRO_RAW 12
#define IMU_STATIONARY_CONFIRM_MS 400u
/* DEPRECATED: heading rotation sign for field-frame accel integration (cosf/sinf removed). */
/* #define IMU_HEADING_ROT_SIGN (-1.0f) */
#define IMU_DEBUG_STREAM_PERIOD_MS 250u
#define ALIGN_STABLE_SAMPLE_COUNT 3u
/* RealignedEvent EventParam: sensor path runs IMU zero + ref re-anchor in Navigate. */
#define ALIGN_REALIGNED_SOURCE_MANUAL (0u)
#define ALIGN_REALIGNED_SOURCE_SENSOR (1u)
#define DISTANCE_FORWARD_TO_ISZ_IN 10.0f
#define DISTANCE_REVERSE_TO_SHOOT_IN 3.0f

#define BNO055_AXIS_X 0u
#define BNO055_AXIS_Y 1u
#define BNO055_AXIS_Z 2u
/* BNO055 mounted vertically: use the sensor Y-axis as the robot heading axis. */
#define BNO055_HEADING_AXIS BNO055_AXIS_Y

/* HW-201 tape sensors: OUT is HIGH off tape and LOW on black tape. */
#define TAPE_SENSOR_1_PORT PORTZ
#define TAPE_SENSOR_1_PIN  PIN4
#define TAPE_SENSOR_2_PORT PORTZ
#define TAPE_SENSOR_2_PIN  PIN5
#define TAPE_SENSOR_3_PORT PORTZ
#define TAPE_SENSOR_3_PIN  PIN6
#define TAPE_SENSOR_4_PORT PORTZ
#define TAPE_SENSOR_4_PIN  PIN8
#define TAPE_SENSOR_5_PORT PORTZ
#define TAPE_SENSOR_5_PIN  PIN9
#define TAPE_SENSOR_PORTZ_PINS (PIN4 | PIN5 | PIN6 | PIN8 | PIN9)
/* Active level: 1 = pin HIGH when on tape, 0 = pin LOW when on tape. */
#define TAPE_BLACK_IS_HIGH 0
#define BEACON_ADC_PIN AD_PORTV8

/* Beacon detector peak output is 1.65V-3.0V into the Uno32 ADC.
 * 1.65V sits near ADC 512 past 16 ft, and 3.0V sits near ADC 930 below 6 ft.
 * [FLAG][#1] Replace the linear placeholder with measured calibration data
 * after testing the detector at 6 ft through 16 ft. */
#define BEACON_AVERAGE_SAMPLE_COUNT 10u
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
#define BUMP_SENSOR_1_PIN  PIN5
#define BUMP_SENSOR_2_PORT PORTV
#define BUMP_SENSOR_2_PIN  PIN6
#define BUMP_SENSOR_3_PORT PORTV
#define BUMP_SENSOR_3_PIN  PIN7
#define BUMP_SENSOR_4_PORT PORTZ
#define BUMP_SENSOR_4_PIN  PIN11
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
#define BEACON_ADC_DELTA 20u

/*
 * L298N H-bridge 1 controls front motors:
 *   OUT1 -> motor FR+, OUT2 -> motor FR-
 *   OUT3 -> motor FL+, OUT4 -> motor FL-
 * L298N H-bridge 2 controls rear motors:
 *   OUT1 -> motor RR+, OUT2 -> motor RR-
 *   OUT3 -> motor RL+, OUT4 -> motor RL-
 */
#define MOTOR_FR_PWM_PIN PWM_PORTZ06
#define MOTOR_FL_PWM_PIN PWM_PORTY12
#define MOTOR_RR_PWM_PIN PWM_PORTY10
#define MOTOR_RL_PWM_PIN PWM_PORTY04
#define SHOOTER_MOTOR_PWM_PIN PWM_PORTX11
#define DRIVE_MOTOR_PWM_PINS (MOTOR_FL_PWM_PIN | MOTOR_FR_PWM_PIN | \
    MOTOR_RL_PWM_PIN | MOTOR_RR_PWM_PIN)
#define ALL_PWM_PINS (MOTOR_FL_PWM_PIN | MOTOR_FR_PWM_PIN | MOTOR_RL_PWM_PIN | \
    MOTOR_RR_PWM_PIN | SHOOTER_MOTOR_PWM_PIN)

/* H-bridge 1 channel A: FR, ENA Z6, IN1 Y8, IN2 W3. */
#define MOTOR_FR_IN1_TRIS PORTY08_TRIS
#define MOTOR_FR_IN1_LAT PORTY08_LAT
#define MOTOR_FR_IN2_TRIS PORTW03_TRIS
#define MOTOR_FR_IN2_LAT PORTW03_LAT

/* H-bridge 1 channel B: FL, ENB Y12, IN3 Y9, IN4 W4. */
#define MOTOR_FL_IN1_TRIS PORTY09_TRIS
#define MOTOR_FL_IN1_LAT PORTY09_LAT
#define MOTOR_FL_IN2_TRIS PORTW04_TRIS
#define MOTOR_FL_IN2_LAT PORTW04_LAT

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

/* [FLAG][#1] Stepper pins are reserved for future launcher/feeder hardware. Update if your stepper board uses a different interface. */
#define STEPPER_STEP_TRIS PORTZ03_TRIS
#define STEPPER_STEP_LAT PORTZ03_LAT
#define STEPPER_DIR_TRIS PORTZ04_TRIS
#define STEPPER_DIR_LAT PORTZ04_LAT
#define STEPPER_ENABLE_TRIS PORTZ05_TRIS
#define STEPPER_ENABLE_LAT PORTZ05_LAT
#define STEPPER_ENABLE_ACTIVE_LOW 1

/* [FLAG][#1] Flip any polarity from 1 to -1 after the real mecanum drivetrain is wired. */
#define MOTOR_FL_POLARITY 1
#define MOTOR_FR_POLARITY 1
#define MOTOR_RL_POLARITY 1
#define MOTOR_RR_POLARITY 1

#define LAUNCHER_SERVO_PIN RC_PORTZ08
#define LAUNCHER_SERVO_MIN_PULSE_US 1000u
#define LAUNCHER_SERVO_MAX_PULSE_US 2000u
#define LAUNCHER_SERVO_CENTER_PULSE_US 1500u
#define SHOOTER_MOTOR_DUTY 900u

#define BNO055_I2C_ADDRESS 0x28u
#define BNO055_I2C_BAUD_HZ 100000u

/* BNO055 I2C hookup on the Uno32 stack. */
#define BNO055_SDA_PIN_LABEL "V3"
#define BNO055_SCL_PIN_LABEL "V4"
#define BNO055_PIN_LABEL "SDA -> V3 digital I/O, SCL -> V4 digital I/O, heading axis -> sensor Y"
#define BEACON_ADC_PIN_LABEL "V8 analog input"
#define TAPE_SENSOR_1_PIN_LABEL "HW-201 OUT -> Z4 digital input, LOW on tape"
#define TAPE_SENSOR_2_PIN_LABEL "HW-201 OUT -> Z5 digital input, LOW on tape"
#define TAPE_SENSOR_3_PIN_LABEL "[FLAG][#1] HW-201 OUT -> Z6 digital input, LOW on tape; conflicts with FR motor PWM"
#define TAPE_SENSOR_4_PIN_LABEL "[FLAG][#1] HW-201 OUT -> Z8 digital input, LOW on tape; conflicts with launcher servo"
#define TAPE_SENSOR_5_PIN_LABEL "HW-201 OUT -> Z9 digital input, LOW on tape"
/* Solenoid pin labels intentionally disabled for now.
 * #define SOLENOID_SENSOR_1_PIN_LABEL ""
 * #define SOLENOID_SENSOR_2_PIN_LABEL ""
 * #define SOLENOID_SENSOR_3_PIN_LABEL ""
 * #define SOLENOID_SENSOR_4_PIN_LABEL ""
 * #define SOLENOID_SENSOR_5_PIN_LABEL ""
 * #define SOLENOID_SENSOR_6_PIN_LABEL ""
 */
#define BUMP_SENSOR_1_PIN_LABEL "FR bump OUT -> V5 digital input, HIGH when bumped"
#define BUMP_SENSOR_2_PIN_LABEL "FL bump OUT -> V6 digital input, HIGH when bumped"
#define BUMP_SENSOR_3_PIN_LABEL "RR bump OUT -> V7 digital input, HIGH when bumped"
#define BUMP_SENSOR_4_PIN_LABEL "RL bump OUT -> Z11 digital input, HIGH when bumped"
#define MOTOR_FL_PIN_LABEL "H-bridge 1 B: ENB PWM Y12, IN3 Y9, IN4 W4, OUT3 FL+, OUT4 FL-"
#define MOTOR_FR_PIN_LABEL "H-bridge 1 A: ENA PWM Z6, IN1 Y8, IN2 W3, OUT1 FR+, OUT2 FR-"
#define MOTOR_RL_PIN_LABEL "H-bridge 2 B: ENB PWM Y4, IN3 Z3, IN4 W6, OUT3 RL+, OUT4 RL-"
#define MOTOR_RR_PIN_LABEL "H-bridge 2 A: ENA PWM Y10, IN1 Y11, IN2 W5, OUT1 RR+, OUT2 RR-"
#define LAUNCHER_SERVO_PIN_LABEL "RC servo Z8"
#define SHOOTER_MOTOR_PIN_LABEL "TIP122 input -> PWM X11"
#define SHOOTER_MOTOR_ADC_PIN_LABEL "[FLAG][#1] remap; W5 is H-bridge 2 RR IN2"
#define STEPPER_PIN_LABEL "[FLAG][#1] STEP Z3 conflicts with drive RL IN3; remap before HW_STEPPER=1"

#endif /* ROBOT_PINS_H */
