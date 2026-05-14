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

#define MOTOR_SPEED_IPS 8.0f
#define ALIGN_SPEED_IPS 3.0f
#define TURN_SPEED_IPS 4.0f
#define MOTOR_DUTY_PER_IPS 70.0f

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

/* Tape sensors: digital inputs on PORTV (all 5 on same port for efficient read).
 * W6 and W8 are reserved for the BNO055 I2C hookup below. */
#define TAPE_SENSOR_1_PORT PORTV
#define TAPE_SENSOR_1_PIN  PIN3
#define TAPE_SENSOR_2_PORT PORTV
#define TAPE_SENSOR_2_PIN  PIN4
#define TAPE_SENSOR_3_PORT PORTV
#define TAPE_SENSOR_3_PIN  PIN5
#define TAPE_SENSOR_4_PORT PORTV
#define TAPE_SENSOR_4_PIN  PIN6
#define TAPE_SENSOR_5_PORT PORTV
#define TAPE_SENSOR_5_PIN  PIN7
#define TAPE_SENSOR_PORTV_PINS (PIN3 | PIN4 | PIN5 | PIN6 | PIN7)
/* Active level: 1 = pin HIGH when on tape, 0 = pin LOW when on tape. */
#define TAPE_BLACK_IS_HIGH 1
#define BEACON_ADC_PIN AD_PORTW7

/* [FLAG][#1] Replace these solenoid ADC placeholders with final wiring or mux channels. */
#define SOLENOID_SENSOR_1_ADC_PIN AD_PORTV3
#define SOLENOID_SENSOR_2_ADC_PIN AD_PORTV4
#define SOLENOID_SENSOR_3_ADC_PIN AD_PORTV5
#define SOLENOID_SENSOR_4_ADC_PIN AD_PORTV6
#define SOLENOID_SENSOR_5_ADC_PIN AD_PORTV7
#define SOLENOID_SENSOR_6_ADC_PIN AD_PORTV8

/* [FLAG][#1] Replace these bump ADC placeholders with final wiring or mux channels. */
#define BUMP_SENSOR_1_ADC_PIN AD_PORTW3
#define BUMP_SENSOR_2_ADC_PIN AD_PORTW4
#define BUMP_SENSOR_3_ADC_PIN AD_PORTW5
#define BUMP_SENSOR_4_ADC_PIN AD_PORTW7

/* [FLAG][#1] Confirm if this is really an ADC command input; shooter motor output uses PWM below. */
#define SHOOTER_MOTOR_ADC_PIN AD_PORTW5

#define SOLENOID_SENSOR_ADC_PINS (SOLENOID_SENSOR_1_ADC_PIN | \
    SOLENOID_SENSOR_2_ADC_PIN | SOLENOID_SENSOR_3_ADC_PIN | \
    SOLENOID_SENSOR_4_ADC_PIN | SOLENOID_SENSOR_5_ADC_PIN | \
    SOLENOID_SENSOR_6_ADC_PIN)
#define BUMP_SENSOR_ADC_PINS (BUMP_SENSOR_1_ADC_PIN | BUMP_SENSOR_2_ADC_PIN | \
    BUMP_SENSOR_3_ADC_PIN | BUMP_SENSOR_4_ADC_PIN)

#define ALL_ROBOT_ADC_PINS (BEACON_ADC_PIN | SOLENOID_SENSOR_1_ADC_PIN | \
    SOLENOID_SENSOR_2_ADC_PIN | SOLENOID_SENSOR_3_ADC_PIN | \
    SOLENOID_SENSOR_4_ADC_PIN | SOLENOID_SENSOR_5_ADC_PIN | \
    SOLENOID_SENSOR_6_ADC_PIN | BUMP_SENSOR_1_ADC_PIN | \
    BUMP_SENSOR_2_ADC_PIN | BUMP_SENSOR_3_ADC_PIN | BUMP_SENSOR_4_ADC_PIN | \
    SHOOTER_MOTOR_ADC_PIN)

/* [FLAG][#1] Confirm whether solenoid / bump active sensors read high. */
#define SOLENOID_ON_ADC_THRESHOLD 700u
#define SOLENOID_ON_IS_HIGH 1
#define BUMP_ON_ADC_THRESHOLD 700u
#define BUMP_ON_IS_HIGH 1
#define BEACON_ADC_DELTA 20u

/* [FLAG][#1] Replace all motor PWM and L298 direction placeholders with final Uno32 stack pins. */
#define MOTOR_FL_PWM_PIN PWM_PORTZ06
#define MOTOR_FR_PWM_PIN PWM_PORTY12
#define MOTOR_RL_PWM_PIN PWM_PORTY10
#define MOTOR_RR_PWM_PIN PWM_PORTY04
#define SHOOTER_MOTOR_PWM_PIN PWM_PORTX11
#define DRIVE_MOTOR_PWM_PINS (MOTOR_FL_PWM_PIN | MOTOR_FR_PWM_PIN | \
    MOTOR_RL_PWM_PIN | MOTOR_RR_PWM_PIN)
#define ALL_PWM_PINS (MOTOR_FL_PWM_PIN | MOTOR_FR_PWM_PIN | MOTOR_RL_PWM_PIN | \
    MOTOR_RR_PWM_PIN | SHOOTER_MOTOR_PWM_PIN)

#define MOTOR_FL_IN1_TRIS PORTX03_TRIS
#define MOTOR_FL_IN1_LAT PORTX03_LAT
#define MOTOR_FL_IN2_TRIS PORTX04_TRIS
#define MOTOR_FL_IN2_LAT PORTX04_LAT

#define MOTOR_FR_IN1_TRIS PORTX05_TRIS
#define MOTOR_FR_IN1_LAT PORTX05_LAT
#define MOTOR_FR_IN2_TRIS PORTX06_TRIS
#define MOTOR_FR_IN2_LAT PORTX06_LAT

#define MOTOR_RL_IN1_TRIS PORTX07_TRIS
#define MOTOR_RL_IN1_LAT PORTX07_LAT
#define MOTOR_RL_IN2_TRIS PORTX08_TRIS
#define MOTOR_RL_IN2_LAT PORTX08_LAT

#define MOTOR_RR_IN1_TRIS PORTY03_TRIS
#define MOTOR_RR_IN1_LAT PORTY03_LAT
#define MOTOR_RR_IN2_TRIS PORTY05_TRIS
#define MOTOR_RR_IN2_LAT PORTY05_LAT

#define SHOOTER_MOTOR_DIR_TRIS PORTY06_TRIS
#define SHOOTER_MOTOR_DIR_LAT PORTY06_LAT

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

/* BNO055 I2C hookup verified on the Uno32 stack. */
#define BNO055_SDA_PIN_LABEL "W6"
#define BNO055_SCL_PIN_LABEL "W8"
#define BNO055_PIN_LABEL "SDA -> W6, SCL -> W8"
#define BEACON_ADC_PIN_LABEL "W7 analog input"
#define TAPE_SENSOR_1_PIN_LABEL "V3 digital input"
#define TAPE_SENSOR_2_PIN_LABEL "V4 digital input"
#define TAPE_SENSOR_3_PIN_LABEL "V5 digital input"
#define TAPE_SENSOR_4_PIN_LABEL "V6 digital input"
#define TAPE_SENSOR_5_PIN_LABEL "V7 digital input"
#define SOLENOID_SENSOR_1_PIN_LABEL "V3 analog input"
#define SOLENOID_SENSOR_2_PIN_LABEL "V4 analog input"
#define SOLENOID_SENSOR_3_PIN_LABEL "V5 analog input"
#define SOLENOID_SENSOR_4_PIN_LABEL "V6 analog input"
#define SOLENOID_SENSOR_5_PIN_LABEL "V7 analog input"
#define SOLENOID_SENSOR_6_PIN_LABEL "V8 analog input"
#define BUMP_SENSOR_1_PIN_LABEL "W3 analog input"
#define BUMP_SENSOR_2_PIN_LABEL "W4 analog input"
#define BUMP_SENSOR_3_PIN_LABEL "W5 analog input"
#define BUMP_SENSOR_4_PIN_LABEL "W7 analog input"
#define MOTOR_FL_PIN_LABEL "PWM Z6, DIR X3/X4"
#define MOTOR_FR_PIN_LABEL "PWM Y12, DIR X5/X6"
#define MOTOR_RL_PIN_LABEL "PWM Y10, DIR X7/X8"
#define MOTOR_RR_PIN_LABEL "PWM Y4, DIR Y3/Y5"
#define LAUNCHER_SERVO_PIN_LABEL "RC servo Z8"
#define SHOOTER_MOTOR_PIN_LABEL "PWM X11, DIR Y6"
#define SHOOTER_MOTOR_ADC_PIN_LABEL "W5 analog input"
#define STEPPER_PIN_LABEL "STEP Z3, DIR Z4, ENABLE Z5"

#endif /* ROBOT_PINS_H */
