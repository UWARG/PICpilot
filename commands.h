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
#define TRIGGER_CAMERA 44
#define SET_TRIGGER_DISTANCE 45
#define SET_GIMBLE_OFFSET 46
#define KILL_PLANE 47
#define UNKILL_PLANE 48

//Multipart Commands
#define NEW_WAYPOINT 128
#define INSERT_WAYPOINT 129
#define SET_RETURN_HOME_COORDINATES 130
#define TARE_IMU 131
#define SET_IMU 132

//Multipart Command Structs
typedef struct _newwaypointwrapper {
    long double longitude;  //TODO: Longitude and Latitude is bulky. If problems arise, change the format.
    long double latitude;
    float altitude;
    float radius; //Radius of turn
} NewWaypointWrapper;
#endif	/* COMMANDS_H */

