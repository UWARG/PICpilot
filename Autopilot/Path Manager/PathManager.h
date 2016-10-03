/* 
 * File:   PathManger.h
 * Author: Chris Hajduk
 *
 * Created on February 4, 2014, 9:13 PM
 */

#ifndef PATHMANGER_H
#define	PATHMANGER_H

//Include
#include "../Common/Common.h"

//Constants
    
//In Radians or (90 degrees)
#define MAX_PATH_APPROACH_ANGLE PI/2 

#define PATH_BUFFER_SIZE 50
#define PATH_FREE 0
#define PATH_FULL 1

#define DUBINS_PATH FALSE

#define PATH 0
#define ORBIT 1

//Waypoint types
#define DEFAULT_WAYPOINT 0
#define PROBE_DROP_WAYPOINT 1
#define HOLD_WAYPOINT 2

//Structs and typedefs

//Function Prototypes
//TODO:Add descriptions to all the function prototypes
void pathManagerInit(void);
void pathManagerRuntime(void);

char followWaypoints(PathData* currentWaypoint, float* position, float heading, int* sp_Heading);
int followLineSegment(PathData* currentWaypoint, float* position, float heading);
int followLastLineSegment(PathData* currentWaypoint, float* position, float heading);
float orbitWaypoint(float* center, float radius, char direction, float* position, float heading);
float followOrbit(float* center, float radius, char direction, float* position, float heading);
float followStraightPath(float* waypointDirection, float* targetWaypoint, float* position, float heading);
float maintainAltitude(PathData* cPath);
void getCoordinates(long double longitude, long double latitude, float* xyCoordinates);
int calculateHeadingHome(PathData home, float* position, float heading);
PathData* initializePathNode(void);
unsigned int destroyPathNode(PathData* node);
PathData* initializePathNodeAndNext(void);
unsigned int appendPathNode(PathData* node);
unsigned int updatePathNode(PathData* node, unsigned int ID);
unsigned int removePathNode(unsigned int ID);
void clearPathNodes(void);
unsigned int insertPathNode(PathData* node, unsigned int previousID, unsigned int nextID);
void copyGPSData(void);
char gpsErrorCheck(double lat, double lon);
void checkAMData(void);
float getWaypointChecksum(void);


#endif	/* PATHMANGER_H */

