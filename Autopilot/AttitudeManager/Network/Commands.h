/*
 * File:   commands.h
 * Author: andrew
 *
 * Created on March 8, 2014, 3:17 AM
 */

#ifndef COMMANDS_H
#define	COMMANDS_H

#define DEBUG_TEST 0

typedef const enum {
    SET_HEADING_GAIN = 1,
    SET_ALTITUDE_GAIN = 2,
    SET_GROUND_SPEED_GAIN = 3,
            
    //unused 15
            
    SET_PATH_GAIN = 19,
    SET_ORBIT_GAIN = 20,
    SET_PITCH_RATE = 22,
    SET_ROLL_RATE = 23,
    SET_YAW_RATE = 24,
    SET_PITCH_ANGLE = 25,
    SET_ROLL_ANGLE = 26,
    SET_YAW_ANGLE = 27,
    SET_ALTITUDE = 28,
    SET_HEADING = 29,
    SET_THROTTLE = 30,
    SET_AUTONOMOUS_LEVEL = 31,
    SET_ANGULAR_WALK_VARIANCE = 32,
    SET_GYRO_VARIANCE = 33,
    SET_MAGNETIC_VARIANCE = 34,
    SET_ACCEL_VARIANCE = 35,
    SET_SCALE_FACTOR = 36,
    CALIBRATE_ALTIMETER = 37,
    CLEAR_WAYPOINTS = 38,
    REMOVE_WAYPOINT = 39,
    SET_TARGET_WAYPOINT = 40,
    RETURN_HOME = 41,
    CANCEL_RETURN_HOME = 42,
    SEND_HEARTBEAT = 43,
    
    //unused 3
            
    KILL_PLANE = 47,
    UNKILL_PLANE = 48,
    UNUSED_49 = 49,
    ARM_VEHICLE = 50,
    DEARM_VEHICLE = 51,
    SET_FLAP = 52,
            
    //unused 6
            
    FOLLOW_PATH = 58,
    EXIT_HOLD_ORBIT = 59,
    SHOW_SCALED_PWM = 60,
    REMOVE_LIMITS = 61,

    //Multi-part Commands
    NEW_WAYPOINT = 128,
    INSERT_WAYPOINT = 129,
    SET_RETURN_HOME_COORDINATES = 130,
    TARE_IMU = 131,
    SET_IMU = 132,
    UPDATE_WAYPOINT = 136,
    SET_ROLL_RATE_GAINS = 137,
    SET_PITCH_RATE_GAINS = 138,
    SET_YAW_RATE_GAINS = 139,
    SET_ROLL_ANGLE_GAINS = 140,
    SET_PITCH_ANGLE_GAINS = 141,
} COMMAND_TYPE;


/**
 * Converts command data into the specified type. Saves on typing. Make sure the data is byte aligned before
 * calling this or you'll get an OPCODE reset. ie. you cant cast a 3 byte array
 * into a 2 byte int if the starting index is 1
 */
#define CMD_TO_INT(data) (*((int*)data))
#define CMD_TO_FLOAT(data) (*((float*)data))
#define CMD_TO_FLOAT_ARRAY(data) ((float*)data)
#define CMD_TO_TYPE(data, type) (*((type*)data))

#endif	/* COMMANDS_H */
