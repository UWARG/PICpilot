/*
 * File:   PathManager.c
 * Author: Chris Hajduk
 *
 * Created on February 4, 2014, 9:13 PM
 */

#include "main.h"
#include "PathManager.h"

#if !(PATH_MANAGER && ATTITUDE_MANAGER && COMMUNICATION_MANAGER)
#include "InterchipDMA.h"
#endif

#if DEBUG
#include <stdio.h>
#include <stdlib.h>
#include "UART1.h"
#endif

#if PATH_MANAGER
extern GPSData gpsData;
extern PMData pmData;
extern newGPSDataAvailable;


float k_gain[2] = {1, 1};

unsigned int currentIndex = 0;
unsigned int nodeID = 0;

char inOrbit = 0;

PathData * path[PATH_BUFFER_SIZE];

void pathManagerInit(void) {
#if DEBUG
    InitUART1();
#endif

    //Communication with GPS
        init_SPI2();
        init_DMA2();

    //Interchip Communication
#if !ATTITUDE_MANAGER
    init_SPI1();
    init_DMA0();
    init_DMA1();
    DMA1REQbits.FORCE = 1;
    while (DMA1REQbits.FORCE == 1);


#endif

    //Initialize first path nodes
    path[0] = initializePathNodeAndNext();
    path[0]->index = 0;
    path[1]->previous = path[0];
    path[1] = path[0]->next;
    path[1]->index = 1;

}

void pathManagerRuntime(void) {

#if DEBUG
        char str[16];
        sprintf(&str,"%f",pmData.time);
        UART1_SendString(&str);
#endif
    //Get GPS data
    if (newGPSDataAvailable){
        newGPSDataAvailable = 0;
        pmData.time = gpsData.time;
        pmData.altitude = gpsData.altitude;
        pmData.longitude = gpsData.longitude;
        pmData.latitude = gpsData.latitude;
        pmData.heading = gpsData.heading;
        pmData.speed = gpsData.speed;
        pmData.satellites = (char)gpsData.satellites;
        pmData.positionFix = (char)gpsData.positionFix;
    }

    float position[3];
    float waypointPosition[3];
    float waypointDirection[3];

    //Get the position of the plane (in meters)
    getCoordinates(gpsData.longitude,gpsData.latitude,&position);
    position[2] = gpsData.altitude;

    if (path[currentIndex]->next != 0){
//        currentIndex = followWaypoints(path[currentIndex],&waypointPosition,&waypointDirection, &position);
    }



    if (inOrbit){
//        pmData.sp_Heading = followOrbit(path[currentIndex]->longitude, path[currentIndex]->latitude, path[currentIndex]->radius, path[currentIndex]->direction);
//        pmData.sp_Altitude = path[currentIndex]->altitude;
    }
    else{
//       pmData.sp_Heading = followStraightPath();
//       pmData.sp_Altitude = maintainAltitude();
    }

    //    //Straight Path Following
    //    long double pathSlope = (path[currentIndex]->next->latitude - path[currentIndex]->latitude)/(path[currentIndex]->next->longitude - path[currentIndex]->longitude);
    //    //Using the first checkpoint as the origin
    //
    //    float dDistance = getDistance(path[currentIndex]->longitude,path[currentIndex]->latitiude,path[currentIndex]->next->longitude,path[currentIndex]->next->latitiude);
    //    float dAltitude = path[currentIndex]->next->altitude - path[currentIndex]->altitude;
    //
    //    long double perpendicularXCoordinate = (gpsData.latitude + gpsData.longitude/pathSlope)/(pathSlope + 1/pathSlope);
    //    float pathError = getDistance(perpendicularXCoordinate, pathSlope * perpendicularXCoordinate, gpsData.longitude, gpsData.latitude);
    //
    //    pmData.sp_Heading = atan(k_gain * pathError) * 180.0/PI; //Gain multiplied by distance from the target path
    //
    //    pmData.sp_Altitude =  getDistance(path[currentIndex]->next->longitude, path[currentIndex]->next->latitude, gpsData.longitude, gpsData.latitude)/dDistance * dAltitude + path[currentIndex]->altitude;


}
char followWaypoints(PathData* currentWaypoint, float* waypointPosition, float* waypointDirection, float* position){
        getCoordinates(currentWaypoint->longitude, currentWaypoint->latitude,&waypointPosition);
        waypointPosition[2] = currentWaypoint->altitude;

        PathData* targetWaypoint = currentWaypoint->next;
        float targetCoordinates[3];
        getCoordinates(targetWaypoint->longitude, targetWaypoint->longitude, &targetCoordinates);
        targetCoordinates[2] = targetWaypoint->altitude;
                
        PathData* nextWaypoint = targetWaypoint->next;
        float nextCoordinates[3];
        getCoordinates(nextWaypoint->longitude, nextWaypoint->longitude, &nextCoordinates);
        nextCoordinates[2] = nextWaypoint->altitude;


        float norm = sqrt(pow(targetCoordinates[0] - waypointPosition[0],2) + pow(targetCoordinates[1] - waypointPosition[1],2) + pow(targetCoordinates[2] - waypointPosition[2],2));
        waypointDirection[0] = (targetCoordinates[0] - waypointPosition[0])/norm;
        waypointDirection[1] = (targetCoordinates[1] - waypointPosition[1])/norm;
        waypointDirection[2] = (targetCoordinates[2] - waypointPosition[2])/norm;

        float nextWaypointDirection[3];
        norm = sqrt(pow(nextCoordinates[0] - targetCoordinates[0],2) + pow(nextCoordinates[1] - targetCoordinates[1],2) + pow(nextCoordinates[2] - targetCoordinates[2],2));
        nextWaypointDirection[0] = (nextCoordinates[0] - targetCoordinates[0])/norm;
        nextWaypointDirection[1] = (nextCoordinates[1] - targetCoordinates[1])/norm;
        nextWaypointDirection[2] = (nextCoordinates[2] - targetCoordinates[2])/norm;

        float normal[3];
        norm = sqrt(pow(waypointDirection[0] + nextWaypointDirection[0],2) + pow(waypointDirection[1] + nextWaypointDirection[1],2) + pow(waypointDirection[2] + nextWaypointDirection[2],2));
        normal[0] = (waypointDirection[0] + nextWaypointDirection[0])/norm;
        normal[1] = (waypointDirection[1] + nextWaypointDirection[1])/norm;
        normal[2] = (waypointDirection[2] + nextWaypointDirection[2])/norm;

        float eucNormal = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
        float eucPosition = sqrt(pow(position[0] - targetCoordinates[0],2) + pow(position[1] - targetCoordinates[1],2) + pow(position[2] - targetCoordinates[2],2));
        float eucSum = sqrt(pow(normal[0] + position[0],2) + pow(normal[1] + position[1],2) + pow(normal[2] + position[2],2));

        if ((normal[0] >= 0 &&  normal[1] >=0)||(normal[0] < 0 && normal[1] < 0))
            eucNormal *= -1;
        if ((position[0] - targetCoordinates[0] >= 0 &&  position[1] - targetCoordinates[1] >=0)||(position[0] - targetCoordinates[0] < 0 && position[1] - targetCoordinates[1] < 0))
            eucNormal *= -1;

        float cosC = (pow(eucSum,2) - pow(eucNormal,2) - pow(eucPosition,2))/(-2 * eucNormal * eucPosition);

        if (eucNormal * eucPosition * cosC > 0){
            return targetWaypoint->index;
        }

        return currentWaypoint->index;

}

float followOrbit(float* center, float radius, char direction, float* position, float heading){
    float orbitDistance = sqrt(pow(position[0] - center[0],2) + pow(position[1] - center[1],2));
    float courseAngle = atan2(position[1] - center[1], position[0] - center[0]); // (y,x) format
    while (courseAngle - heading < -PI){
        courseAngle += 2 * PI;
    }
    while (courseAngle - heading > PI){
        courseAngle -= 2 * PI;
    }

    return courseAngle + direction * (PI/2 + atan(k_gain[ORBIT] * (orbitDistance - radius)/radius));
}
float followStraightPath(float* waypointPosition, float* waypointDirection, float* position, float heading){
    float courseAngle = atan2(waypointDirection[1], waypointDirection[0]); // (y,x) format
    while (courseAngle - heading < -PI){
        courseAngle += 2 * PI;
    }
    while (courseAngle - heading > PI){
        courseAngle -= 2 * PI;
    }
    float pathError = -sin(waypointPosition[0] - position[0]) + cos(waypointPosition[1] - position[1]);
    return (courseAngle - heading) * 2/PI * atan(k_gain[PATH] * pathError) * 180.0/PI;

}
//float followStraightPath(float longitude1, float latitude1, long double longitude2, long double latitude2){
//        //Straight Path Following
//        long double pathSlope = (latitude2 - latitude1)/(longitude2 - longitude1);
//        //Using the first checkpoint as the origin
//
//        long double perpendicularXCoordinate = (gpsData.latitude + gpsData.longitude/pathSlope)/(pathSlope + 1/pathSlope);
//        float pathError = getDistance(perpendicularXCoordinate, pathSlope * perpendicularXCoordinate, gpsData.longitude, gpsData.latitude);
//        return atan(k_gain[PATH] * pathError) * 180.0/PI;
//}

float maintainAltitude(PathData* cPath){
    float dAltitude = cPath->next->altitude - cPath->altitude;
    float dDistance = getDistance(cPath->longitude, cPath->latitude, cPath->next->longitude, cPath->next->latitude);
    return getDistance(cPath->next->longitude, cPath->next->latitude, gpsData.longitude, gpsData.latitude)/dDistance * dAltitude + cPath->altitude;
}

void getCoordinates(long double longitude, long double latitude, float* xyCoordinates){
    xyCoordinates[0] = getDistance(RELATIVE_LATITUDE, RELATIVE_LONGITUDE, RELATIVE_LATITUDE, longitude);//Longitude relative to (0,0)
    xyCoordinates[1] = getDistance(RELATIVE_LATITUDE, RELATIVE_LONGITUDE, latitude, RELATIVE_LONGITUDE);
}

unsigned int getIndexFromID(unsigned int ID) {

    return 0;
}

PathData* initializePathNode(void) {
//    if (nodeIndex >= PATH_BUFFER_SIZE){
//        nodeIndex = 0;
//    }
    PathData* node = (PathData *) malloc(sizeof (PathData));
    node->id = nodeID++;
    node->next = 0;
    node->previous = 0;
    return node;
}

unsigned int destroyPathNode(PathData* node){
    unsigned int ID = node->id;
    free(node);
    return ID;
}

PathData* initializePathNodeAndNext(void) {
    PathData* temp = initializePathNode();
    temp->next = initializePathNode();
    temp->next->previous = temp;
    return temp;
}

unsigned int appendPathNode(PathData* node){

}
unsigned int removePathNode(unsigned int index){
    unsigned int previousIndex = 0;
    PathData* previousNode = 0;
}
unsigned int insertPathNode(PathData* node, unsigned int index){
    
}
#endif

float getDistance(long double lat1, long double lon1, long double lat2, long double lon2){ //in meters
    long double dLat = deg2rad(lat2 - lat1);
    long double dLon = deg2rad(lon2 - lon1);

    float a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);

    if ((dLat >= 0 && dLon >=0)||(dLat < 0 && dLon < 0)){
        return EARTH_RADIUS * (2 * atan2(sqrt(a),sqrt(1 - a))) * 1000;
    }
    else {
         return EARTH_RADIUS * (2 * atan2(sqrt(a),sqrt(1 - a))) * -1000;
    }
}