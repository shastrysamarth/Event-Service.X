#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

typedef enum {
    BOUNDARY_TOP,
    BOUNDARY_BOTTOM,
} BoundaryChoice_t;

typedef enum {
    MOVEMENT_AXIS_HORIZONTAL,
    MOVEMENT_AXIS_VERTICAL,
} MovementAxis_t;

typedef enum {
    TAPE_SENSOR_1 = 1,
    TAPE_SENSOR_2,
    TAPE_SENSOR_3,
    TAPE_SENSOR_4,
    TAPE_SENSOR_5,
    TAPE_SENSOR_6,
    TAPE_SENSOR_7,
    TAPE_SENSOR_8,
    TAPE_SENSOR_9,
} TapeSensor_t;

typedef enum {
    SOLENOID_SENSOR_1 = 1,
    SOLENOID_SENSOR_2,
    SOLENOID_SENSOR_3,
    SOLENOID_SENSOR_4,
    SOLENOID_SENSOR_5,
    SOLENOID_SENSOR_6,
} SolenoidSensor_t;

typedef enum {
    BUMP_SENSOR_1 = 1,
    BUMP_SENSOR_2,
    BUMP_SENSOR_3,
    BUMP_SENSOR_4,
} BumpSensor_t;

typedef enum {
    TURN_PIVOT_CENTER,
    TURN_PIVOT_FRONT_CENTER,
    TURN_PIVOT_BACK_CENTER,
    TURN_PIVOT_LEFT_CENTER,
    TURN_PIVOT_RIGHT_CENTER,
} TurnPivot_t;

typedef enum {
    DISTANCE_AXIS_X,
    DISTANCE_AXIS_Y,
} DistanceAxis_t;

#endif /* ROBOT_TYPES_H */
