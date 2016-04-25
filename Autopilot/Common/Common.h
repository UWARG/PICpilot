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
//-80.539678
#define RELATIVE_LONGITUDE -71.64781
//43.473004
#define RELATIVE_LATITUDE 48.51031

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
#define PM_CALIBRATE_ALTIMETER 32
#define PM_SET_PATH_GAIN 64
#define PM_SET_ORBIT_GAIN 65

//Structs and typedefs
typedef struct _waypointWrapper{
    long double longitude;  //TODO: Longitude and Latitude is bulky. If problems arise, change the format.
    long double latitude;
    float altitude;
    float radius; //Radius of turn
    char type; //Regular or probe drop location
    char nextId; //For use with insertNode() or operations that require reference to another node
    char previousId; //For use with insertNode() or operations that require reference to another node
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

typedef struct _PMData { //54 Bytes
    float time;     //4 Bytes   -  hhmmss.ssss
    long double latitude;  //8 Bytes - ddd.mmmmmm
    long double longitude; //8 Bytes - ddd.mmmmmm
    float speed;    //KM/H
    float altitude;
    float airspeed;
    float pmPathGain;
    float pmOrbitGain;
    int sp_Altitude; // Meters
    int heading;  //Degrees
    int sp_Heading; //Degrees
    int batteryLevel;
    char satellites;    //1 Byte
    char positionFix;   //0 = No GPS, 1 = GPS fix, 2 = DGSP Fix
    char targetWaypoint;
    char waypointCount;
    char waypointChecksum;
    char pathFollowing;
//    char padding;
    char checkbyteDMA;
} PMData;

typedef struct _AMData { //54 Bytes
    WaypointWrapper waypoint;
    float pathGain;
    float orbitGain;
    float calibrationHeight;
    char command;
    char followPath;
//    char padding1;
//    char padding2;
//    char padding3;
//    char padding4;
//    char padding5;
//    char padding6;
//    char padding7;
//    char padding8;
//    char padding9;
//    char padding10;
//    char padding11;
//    char padding12;
    char checksum;
    char checkbyteDMA;
} AMData;

char generatePMDataDMAChecksum(void);
char generateAMDataDMACheckbyte(void);
char generateAMDataChecksum(AMData* data);
float getDistance(long double lat1, long double lon1, long double lat2, long double lon2);


#endif	/* COMMON_H */

