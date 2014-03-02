/* 
 * File:   PathManger.h
 * Author: Chris Hajduk
 *
 * Created on February 4, 2014, 9:13 PM
 */

#ifndef PATHMANGER_H
#define	PATHMANGER_H

//Constants
#define EARTH_RADIUS 6378.137
#define RELATIVE_LONGITUDE -80.5767466670668
#define RELATIVE_LATITUDE 43.5305800001903

#define PATH_BUFFER_SIZE 100

#define PATH 0
#define ORBIT 1

//Structs and typedefs

typedef struct _PathData{
    struct _PathData* next;
    struct _PathData* previous;
    long double longitude;  //TODO: Longitude and Latitude is bulky. Use cartesian 2D approximations
    long double latitude;
    float altitude;
    float radius; //Radius of turn
    char direction; //Clockwise/AntiClockwise
    char id;    //Array ID
    char index;
} PathData;


//Function Prototypes
void pathManagerInit(void);
void pathManagerRuntime(void);

char followWaypoints(PathData* currentWaypoint, float* waypointPosition, float* waypointDirection, float* position);
float followOrbit(long double longitude, long double latitude, float radius, char direction);
float followStraightPath(float* waypointPosition, float* waypointDirection, float* position, float heading);
float maintainAltitude(PathData* cPath);
void getCoordinates(long double longitude, long double latitude, float* xyCoordinates);
float getDistance(long double lon1, long double lat1, long double lon2, long double lat2);
PathData* initializePathNode(void);
PathData* initializePathNodeAndNext(void);



#endif	/* PATHMANGER_H */

