/* 
 * File:   PathManger.h
 * Author: Chris Hajduk
 *
 * Created on February 4, 2014, 9:13 PM
 */

#ifndef COMMON_H
#define	COMMON_H

#define FCY 40000000

//Includes
#include <xc.h>
#include <libpic30.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//Constants
#define EARTH_RADIUS 6378.137

//Flying Dutchmen: 43.473004, -80.539678
#define RELATIVE_LATITUDE 43.473004
#define RELATIVE_LONGITUDE -80.539678
//Southport, Manitoba: 49.912153, -98.268836
//#define RELATIVE_LATITUDE 49.912153
//#define RELATIVE_LONGITUDE -98.268836

//used for error checking to see if GPS coordinates make sense
#define GPS_ERROR 3 

//Define constants for global use in the code
#define TRUE	0xFF
#define FALSE	0x00

//Mathematical Constants
#define PI 3.14159265

//Basic Mathematical Conversions
#define deg2rad(DEG) ((DEG) * PI/180.0)
#define rad2deg(RAD) ((RAD) * 180.0/PI)

// Datalink --> Path Manager commands
#define PM_DEBUG_TEST 0
#define PM_NEW_WAYPOINT 1
#define PM_CLEAR_WAYPOINTS 2
#define PM_INSERT_WAYPOINT 3
#define PM_UPDATE_WAYPOINT 4
#define PM_REMOVE_WAYPOINT 5
#define PM_SET_TARGET_WAYPOINT 6
#define PM_SET_RETURN_HOME_COORDINATES 7
#define PM_RETURN_HOME 8
#define PM_CANCEL_RETURN_HOME 9
#define PM_FOLLOW_PATH 10
#define PM_EXIT_HOLD_ORBIT 11
#define PM_CALIBRATE_ALTIMETER 32
#define PM_SET_PATH_GAIN 64
#define PM_SET_ORBIT_GAIN 65

//Structs and typedefs

/* For reference: 
 In MPLAB XC 16 compiler:
 char           : 1 byte
 int            : 2 bytes
 long int       : 4 bytes
 float          : 4 bytes
 long double    : 8 bytes
 */


/*  WAYPOINT WRAPPER IS USED FOR NETWORKING 
    DO NOT CHANGE THIS UNLESS YOU KNOW WHAT YOU ARE DOING */
typedef struct _waypointWrapper{ // 28 bytes
    long double longitude; //TODO: Longitude and Latitude is bulky. If problems arise, change the format.
    long double latitude;
    float altitude;
    float radius; //Radius of turn
    char type; //Regular or hold location
    char previousId; //For use with insertNode() or operations that require reference to another node
    char nextId; //For use with insertNode() or operations that require reference to another node
    char id;    //Array ID
} WaypointWrapper;

typedef struct _PathData{
    struct _PathData* next;
    struct _PathData* previous;
    long double longitude;  //TODO: Longitude and Latitude is bulky. Use cartesian 2D approximations
    long double latitude;
    float altitude;
    float radius; //Radius of turn
    char type;
    char id;    //Array ID
    char index;
} PathData;

/* Typing guidelines:
 * When dealing with C-style strings or raw characters, use char
 * When you need an integer no larger than 8 bits (i.e. +127/-128 or 0-255), use (u)int8_t
 * When dealing with raw bytes (i.e. from a serial interface), use byte
 */
typedef uint8_t byte;
typedef uint16_t word;

float getDistance(long double lat1, long double lon1, long double lat2, long double lon2);

/**
 * Limits an input value to a specified range.
 * @param input Pointer to the input value
 * @param min Minimum value of input
 * @param max Maximum value of input
 */
void constrain(int16_t* input, int16_t min, int16_t max);

#endif	/* COMMON_H */

