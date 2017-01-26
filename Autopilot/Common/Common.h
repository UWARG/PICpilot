/* 
 * File:   PathManger.h
 * Author: Chris Hajduk
 *
 * Created on February 4, 2014, 9:13 PM
 */

#ifndef COMMON_H
#define	COMMON_H

//Includes
#include <p33FJ256GP710A.h>
#include <stdio.h>
#include <stdlib.h>
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

//Waypoint Management Commands
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
    char type; //Regular or probe drop location
    char previousId; //For use with insertNode() or operations that require reference to another node
    char nextId; //For use with insertNode() or operations that require reference to another node
    char id;    //Array ID
}WaypointWrapper;

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

typedef struct _PMData { // 62 Bytes
    float time;     // 4 Bytes   -  hhmmss.ssss
    long double latitude;  // 8 Bytes - ddd.mmmmmm
    long double longitude; // 8 Bytes - ddd.mmmmmm
    float speed;    //KM/H
    float altitude;
    float airspeed;
    float pmPathGain;
    float pmOrbitGain;
    float waypointChecksum;
    int sp_Altitude; // Meters
    int heading;  //Degrees
    int sp_Heading; //Degrees
    int batteryLevel1;
    int batteryLevel2;
    char satellites;    //1 Byte
    char positionFix;   //0 = No GPS, 1 = GPS fix, 2 = DGSP Fix
    char targetWaypoint;
    char waypointCount;
    char pathFollowing;
    char dropProbe; //0 = No drop, 1 = 1st drop, 2 - 2nd drop, etc..
    char checkbyteDMA1;
    char checkbyteDMA2;
} PMData;

typedef struct _AMData { // 60 Bytes
    WaypointWrapper waypoint; //28 bytes
    float pathGain;
    float orbitGain;
    float calibrationHeight;
    char command;
    char followPath;
    char padding[16];
    char checksum;
    char checkbyteDMA;
} AMData;

char generatePMDataDMAChecksum1(void);
char generatePMDataDMAChecksum2(void);
char generateAMDataDMACheckbyte(void);
char generateAMDataChecksum(AMData* data);
float getDistance(long double lat1, long double lon1, long double lat2, long double lon2);


#endif	/* COMMON_H */

