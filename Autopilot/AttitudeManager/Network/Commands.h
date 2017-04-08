/*
 * File:   commands.h
 * Author: andrew
 *
 * Created on March 8, 2014, 3:17 AM
 */

#ifndef COMMANDS_H
#define	COMMANDS_H

#define DEBUG_TEST 0
#define SET_PITCH_KD_GAIN 1
#define SET_ROLL_KD_GAIN 2
#define SET_YAW_KD_GAIN 3
#define SET_PITCH_KP_GAIN 4
#define SET_ROLL_KP_GAIN 5
#define SET_YAW_KP_GAIN 6
#define SET_PITCH_KI_GAIN 7
#define SET_ROLL_KI_GAIN 8
#define SET_YAW_KI_GAIN 9
#define SET_HEADING_KD_GAIN 10
#define SET_HEADING_KP_GAIN 11
#define SET_HEADING_KI_GAIN 12
#define SET_ALTITUDE_KD_GAIN 13
#define SET_ALTITUDE_KP_GAIN 14
#define SET_ALTITUDE_KI_GAIN 15
#define SET_THROTTLE_KD_GAIN 16
#define SET_THROTTLE_KP_GAIN 17
#define SET_THROTTLE_KI_GAIN 18
#define SET_PATH_GAIN 19
#define SET_ORBIT_GAIN 20
#define SHOW_GAIN 21
#define SET_PITCH_RATE 22
#define SET_ROLL_RATE 23
#define SET_YAW_RATE 24
#define SET_PITCH_ANGLE 25
#define SET_ROLL_ANGLE 26
#define SET_YAW_ANGLE 27
#define SET_ALTITUDE 28
#define SET_HEADING 29
#define SET_THROTTLE 30
#define SET_AUTONOMOUS_LEVEL 31
#define SET_ANGULAR_WALK_VARIANCE 32
#define SET_GYRO_VARIANCE 33
#define SET_MAGNETIC_VARIANCE 34
#define SET_ACCEL_VARIANCE 35
#define SET_SCALE_FACTOR 36
#define CALIBRATE_ALTIMETER 37
#define CLEAR_WAYPOINTS 38
#define REMOVE_WAYPOINT 39
#define SET_TARGET_WAYPOINT 40
#define RETURN_HOME 41
#define CANCEL_RETURN_HOME 42
#define SEND_HEARTBEAT 43
#define UNUSED_44 44
#define UNUSED_45 45
#define UNUSED_46 46
#define KILL_PLANE 47
#define UNKILL_PLANE 48
#define UNUSED_49 49
#define ARM_VEHICLE 50
#define DEARM_VEHICLE 51
#define SET_FLAP 52
#define UNUSED_53 53
#define UNUSED_54 54
#define UNUSED_55 55
#define UNUSED_56 56
#define UNUSED_57 57
#define FOLLOW_PATH 58
#define EXIT_HOLD_ORBIT 59
#define SHOW_SCALED_PWM 60
#define REMOVE_LIMITS 61

//Multipart Commands
#define NEW_WAYPOINT 128
#define INSERT_WAYPOINT 129
#define SET_RETURN_HOME_COORDINATES 130
#define TARE_IMU 131
#define SET_IMU 132
#define SET_KDVALUES 133
#define SET_KPVALUES 134
#define SET_KIVALUES 135
#define UPDATE_WAYPOINT 136
#define SET_GAINS 137

/**
 * Converts command data into the specified type. Saves on typing. Make sure the data is byte aligned before
 * calling this or you'll get an OPCODE reset. ie. you cant cast a 3 byte array
 * into a 2 byte int if the starting index is 1
 */
#define CMD_TO_INT(data) (*((int*)data))
#define CMD_TO_FLOAT(data) (*((float*)data))
#define CMD_TO_FLOAT_ARRAY(data) ((float*)data)
#define CMD_TO_TYPE(data, type) (*((type*)data))

//Multipart Command Structs

#endif	/* COMMANDS_H */
