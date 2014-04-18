/*
 * File:   PathManager.c
 * Author: Chris Hajduk
 *
 * Created on February 4, 2014, 9:13 PM
 */

#include "main.h"
#include "PathManager.h"
#include "MPL3115A2.h"

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
#if !ATTITUDE_MANAGER
extern AMData amData;
#endif

extern char newGPSDataAvailable;


float k_gain[2] = {1, 1};

unsigned int currentBufferIndex = 0; //Last index that was filled
unsigned int currentNodeID = 0; //Last ID that was used
unsigned int currentIndex = 0; //Current Index that is being followed

char orbitPathStatus = 0;

char lastAMDataChecksum = 0;

PathData* path[PATH_BUFFER_SIZE];
char pathStatus[PATH_BUFFER_SIZE];
char pathCount = 0;

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

    //Communication with Altimeter
    initAltimeter();
#endif

    //Initialize first path nodes
//    PathData* node = initializePathNode();
//    node->altitude = 10;
//    node->latitude = 43.4731655738112;
//    node->longitude = -80.5374240875244;
//    node->radius = 10;
//    appendPathNode(node);
//    node = initializePathNode();
//    node->altitude = 10;
//    node->latitude = 43.4718886758826;
//    node->longitude = -80.5391192436218;
//    node->radius = 10;
//    appendPathNode(node);
}

void pathManagerRuntime(void) {

#if DEBUG
//        char str[16];
//        sprintf(&str,"%f",pmData.time);
//        UART1_SendString(&str);
#endif
    //Get GPS data
    copyGPSData();
#if !ATTITUDE_MANAGER
    //Check for new uplink command data
    checkAMData();
#endif
    float position[3];
    float heading;
    //Get the position of the plane (in meters)
    getCoordinates(gpsData.longitude,gpsData.latitude,(float*)&position);
    position[2] = gpsData.altitude;
    heading = (float)gpsData.heading;

    if (pathCount > 2){
        currentIndex = followWaypoints(path[currentIndex], (float*)&position, heading, (int*)&pmData.sp_Heading);
    }
    else if (pathCount > 1){
        pmData.sp_Heading = followLineSegment(path[currentIndex], (float*)&position, heading);
    }

}
char followWaypoints(PathData* currentWaypoint, float* position, float heading, int* sp_Heading){
        float waypointPosition[3];
        getCoordinates(currentWaypoint->longitude, currentWaypoint->latitude, (float*)&waypointPosition);
        waypointPosition[2] = currentWaypoint->altitude;

        PathData* targetWaypoint = currentWaypoint->next;
        float targetCoordinates[3];
        getCoordinates(targetWaypoint->longitude, targetWaypoint->longitude, (float*)&targetCoordinates);
        targetCoordinates[2] = targetWaypoint->altitude;
                
        PathData* nextWaypoint = targetWaypoint->next;
        float nextCoordinates[3];
        getCoordinates(nextWaypoint->longitude, nextWaypoint->longitude, (float*)&nextCoordinates);
        nextCoordinates[2] = nextWaypoint->altitude;

        float waypointDirection[3];
        float norm = sqrt(pow(targetCoordinates[0] - waypointPosition[0],2) + pow(targetCoordinates[1] - waypointPosition[1],2) + pow(targetCoordinates[2] - waypointPosition[2],2));
        waypointDirection[0] = (targetCoordinates[0] - waypointPosition[0])/norm;
        waypointDirection[1] = (targetCoordinates[1] - waypointPosition[1])/norm;
        waypointDirection[2] = (targetCoordinates[2] - waypointPosition[2])/norm;

        float nextWaypointDirection[3];
        norm = sqrt(pow(nextCoordinates[0] - targetCoordinates[0],2) + pow(nextCoordinates[1] - targetCoordinates[1],2) + pow(nextCoordinates[2] - targetCoordinates[2],2));
        nextWaypointDirection[0] = (nextCoordinates[0] - targetCoordinates[0])/norm;
        nextWaypointDirection[1] = (nextCoordinates[1] - targetCoordinates[1])/norm;
        nextWaypointDirection[2] = (nextCoordinates[2] - targetCoordinates[2])/norm;

        float turningAngle = acos(-deg2rad(waypointDirection[0] * nextWaypointDirection[0] + waypointDirection[1] * nextWaypointDirection[1] + waypointDirection[2] * nextWaypointDirection[2]));

        if (orbitPathStatus == PATH){
            float halfPlane[3];
            halfPlane[0] = targetCoordinates[0] - (targetWaypoint->radius/tan(turningAngle/2)) * waypointDirection[0];
            halfPlane[1] = targetCoordinates[1] - (targetWaypoint->radius/tan(turningAngle/2)) * waypointDirection[1];
            halfPlane[2] = targetCoordinates[2] - (targetWaypoint->radius/tan(turningAngle/2)) * waypointDirection[2];

            float eucNormal = sqrt(halfPlane[0] * halfPlane[0] + halfPlane[1] * halfPlane[1] + halfPlane[2] * halfPlane[2]) * (halfPlane[0] < 0?-1:1) * (halfPlane[1] < 0?-1:1) * (halfPlane[2] < 0?-1:1);
            float eucPosition = sqrt(pow(halfPlane[0] - position[0],2) + pow(halfPlane[1] - position[1],2) + pow(halfPlane[2] - position[2],2)) * ((halfPlane[0] - position[0]) < 0?-1:1) * ((halfPlane[1] - position[1]) < 0?-1:1) * ((halfPlane[2] - position[2]) < 0?-1:1);
            float eucSum = sqrt(pow(2 * halfPlane[0] - position[0],2) + pow(2 * halfPlane[1] - position[1],2) + pow(2 * halfPlane[2] - position[2],2)) * ((2 * halfPlane[0] - position[0]) < 0?-1:1) * ((2 * halfPlane[1] - position[1]) < 0?-1:1) * ((2 * halfPlane[2] - position[2]) < 0?-1:1);
            float cosC = (pow(eucSum,2) - pow(eucNormal,2) - pow(eucPosition,2))/(-2 * eucNormal * eucPosition);
            if (cosC > 0){
                orbitPathStatus = ORBIT;
            }
            *sp_Heading = followStraightPath((float*)&waypointDirection,(float*)position, heading);
        }
        else{
            float halfPlane[3];
            halfPlane[0] = targetCoordinates[0] + (targetWaypoint->radius/tan(turningAngle/2)) * waypointDirection[0];
            halfPlane[1] = targetCoordinates[1] + (targetWaypoint->radius/tan(turningAngle/2)) * waypointDirection[1];
            halfPlane[2] = targetCoordinates[2] + (targetWaypoint->radius/tan(turningAngle/2)) * waypointDirection[2];

            float eucNormal = sqrt(halfPlane[0] * halfPlane[0] + halfPlane[1] * halfPlane[1] + halfPlane[2] * halfPlane[2]) * (halfPlane[0] < 0?-1:1) * (halfPlane[1] < 0?-1:1) * (halfPlane[2] < 0?-1:1);
            float eucPosition = sqrt(pow(halfPlane[0] - position[0],2) + pow(halfPlane[1] - position[1],2) + pow(halfPlane[2] - position[2],2)) * ((halfPlane[0] - position[0]) < 0?-1:1) * ((halfPlane[1] - position[1]) < 0?-1:1) * ((halfPlane[2] - position[2]) < 0?-1:1);
            float eucSum = sqrt(pow(2 * halfPlane[0] - position[0],2) + pow(2 * halfPlane[1] - position[1],2) + pow(2 * halfPlane[2] - position[2],2)) * ((2 * halfPlane[0] - position[0]) < 0?-1:1) * ((2 * halfPlane[1] - position[1]) < 0?-1:1) * ((2 * halfPlane[2] - position[2]) < 0?-1:1);
            float cosC = (pow(eucSum,2) - pow(eucNormal,2) - pow(eucPosition,2))/(-2 * eucNormal * eucPosition);
            if (cosC > 0){
                orbitPathStatus = PATH;
                return targetWaypoint->index;
            }

            char turnDirection = waypointDirection[0] * nextWaypointDirection[1] - waypointDirection[1] * nextWaypointDirection[0]>0?1:-1;
            float euclideanWaypointDirection = sqrt(pow(nextWaypointDirection[0] - waypointDirection[0],2) + pow(nextWaypointDirection[1] - waypointDirection[1],2) + pow(nextWaypointDirection[2] - waypointDirection[2],2)) * ((nextWaypointDirection[0] - waypointDirection[0]) < 0?-1:1) * ((nextWaypointDirection[1] - waypointDirection[1]) < 0?-1:1) * ((nextWaypointDirection[2] - waypointDirection[2]) < 0?-1:1);
            
            float turnCenter[3];
            turnCenter[0] = targetCoordinates[0] + (targetWaypoint->radius/tan(turningAngle/2) * (nextWaypointDirection[0] - waypointDirection[0])/euclideanWaypointDirection);
            turnCenter[1] = targetCoordinates[1] + (targetWaypoint->radius/tan(turningAngle/2) * (nextWaypointDirection[1] - waypointDirection[1])/euclideanWaypointDirection);
            turnCenter[2] = targetCoordinates[2] + (targetWaypoint->radius/tan(turningAngle/2) * (nextWaypointDirection[2] - waypointDirection[2])/euclideanWaypointDirection);
            *sp_Heading = (int)followOrbit((float*) &turnCenter,targetWaypoint->radius, turnDirection, (float*)position, heading);
        }

        return currentWaypoint->index;

}
int followLineSegment(PathData* currentWaypoint, float* position, float heading){
        float waypointPosition[3];
        getCoordinates(currentWaypoint->longitude, currentWaypoint->latitude, (float*)&waypointPosition);
        waypointPosition[2] = currentWaypoint->altitude;

        PathData* targetWaypoint = currentWaypoint->next;
        float targetCoordinates[3];
        getCoordinates(targetWaypoint->longitude, targetWaypoint->latitude, (float*)&targetCoordinates);
        targetCoordinates[2] = targetWaypoint->altitude;


        float waypointDirection[3];
        float norm = sqrt(pow(targetCoordinates[0] - waypointPosition[0],2) + pow(targetCoordinates[1] - waypointPosition[1],2) + pow(targetCoordinates[2] - waypointPosition[2],2));
        waypointDirection[0] = (targetCoordinates[0] - waypointPosition[0])/norm;
        waypointDirection[1] = (targetCoordinates[1] - waypointPosition[1])/norm;
        waypointDirection[2] = (targetCoordinates[2] - waypointPosition[2])/norm;

        return (int)followStraightPath((float*)&waypointDirection, (float*)position, heading);
}

float followOrbit(float* center, float radius, char direction, float* position, float heading){//Heading in degrees (magnetic)
    heading = deg2rad(90 - heading);
    float orbitDistance = sqrt(pow(position[0] - center[0],2) + pow(position[1] - center[1],2));
    float courseAngle = atan2(position[1] - center[1], position[0] - center[0]); // (y,x) format
 
    while (courseAngle - heading < -PI){
        courseAngle += 2 * PI;
    }
    while (courseAngle - heading > PI){
        courseAngle -= 2 * PI;
    }

    return 90 - rad2deg(courseAngle + direction * (PI/2 + atan(k_gain[ORBIT] * (orbitDistance - radius)/radius))); //Heading in degrees (magnetic)
}
float followStraightPath(float* waypointDirection, float* position, float heading){ //Heading in degrees (magnetic)
    heading = deg2rad(90 - heading);//90 - heading = magnetic heading to cartesian heading
    float courseAngle = atan2(waypointDirection[1], waypointDirection[0]); // (y,x) format
    while (courseAngle - heading < -PI){ 
        courseAngle += 2 * PI;
    }
    while (courseAngle - heading > PI){
        courseAngle -= 2 * PI;
    }
    float pathError = -sin(courseAngle) * (position[0] - waypointDirection[0]) + cos(courseAngle) * (position[1] - waypointDirection[1]);
    return 90 - rad2deg(courseAngle - MAX_PATH_APPROACH_ANGLE * 2/PI * atan(k_gain[PATH] * pathError)); //Heading in degrees (magnetic)

}

float maintainAltitude(PathData* cPath){
    float dAltitude = cPath->next->altitude - cPath->altitude;
    float dDistance = getDistance(cPath->longitude, cPath->latitude, cPath->next->longitude, cPath->next->latitude);
    return getDistance(cPath->next->longitude, cPath->next->latitude, gpsData.longitude, gpsData.latitude)/dDistance * dAltitude + cPath->altitude;
}

void getCoordinates(long double longitude, long double latitude, float* xyCoordinates){
    xyCoordinates[0] = getDistance(RELATIVE_LATITUDE, RELATIVE_LONGITUDE, RELATIVE_LATITUDE, longitude);//Longitude relative to (0,0)
    xyCoordinates[1] = getDistance(RELATIVE_LATITUDE, RELATIVE_LONGITUDE, latitude, RELATIVE_LONGITUDE);
}

unsigned int getIndexFromID(unsigned int id) {
    int i = 0;
    for (i = currentBufferIndex; i >= 0; i--){
        if (path[i]->id == id){
            return i;
        }
    }
    //If it isn't found, return -1
    return -1;
}

PathData* initializePathNode(void) {
//    if (nodeIndex >= PATH_BUFFER_SIZE){
//        nodeIndex = 0;
//    }
    PathData* node = (PathData *) malloc(sizeof (PathData));
    node->id = currentNodeID++;
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
    int previousIndex = currentBufferIndex - 1;
    if (previousIndex == -1){
        path[currentBufferIndex] = node;
        node->index = currentBufferIndex++;
        pathCount++;
        return -1;
    }
    PathData* previousNode = path[previousIndex];
    node->previous = previousNode;
    path[currentBufferIndex] = node;
    node->index = currentBufferIndex++;
    pathCount++;
    //Update previous node
    previousNode->next = node;
    return node->id;
    
}
unsigned int removePathNode(unsigned int ID){ //Also attempts to destroys the node.

    unsigned int nodeIndex = getIndexFromID(ID);
    if (nodeIndex == -1){
        return -1;
    }
    PathData* node = path[nodeIndex];
    PathData* previousNode = node->previous;
    PathData* nextNode = node->next;
    previousNode->next = nextNode;
    nextNode->previous = previousNode;

    destroyPathNode(node);
    path[nodeIndex] = 0;
    return ID;
}
void clearPathNodes(void){
    int i = 0;
    for (i = 0; i < PATH_BUFFER_SIZE; i++){
        destroyPathNode(path[i]);
        path[i] = 0;
        pathStatus[i] = PATH_FREE;
    }
    pathCount = 0;
    currentBufferIndex = 0; //Last index that was filled
    currentNodeID = 0; //Last ID that was used
    currentIndex = 0; //Current Index that is being followed
}

unsigned int insertPathNode(PathData* node, unsigned int previousID, unsigned int nextID){
    int nextIndex = getIndexFromID(nextID);
    int previousIndex = getIndexFromID(previousID);
    if (nextIndex == -1 || previousIndex == -1){
        return -1;
    }

    PathData* nextNode = path[nextIndex];
    PathData* previousNode = path[previousIndex];
    //Setup the node object first
    if (node->id == 0){ //If it hasn't been properly initialized.
        node->id = currentNodeID++;
        path[currentBufferIndex++] = node;
    }
    node->next = nextNode;
    node->previous = previousNode;

    //Update previous and next nodes
    nextNode->previous = node;
    previousNode->next = node;

    return node->id;
}

void copyGPSData(){
    if (newGPSDataAvailable){
        newGPSDataAvailable = 0;
        pmData.time = gpsData.time;
        pmData.longitude = gpsData.longitude;
        pmData.altitude = getAltitude(); //gpsData.altitude;
        pmData.latitude = gpsData.latitude;
        pmData.heading = gpsData.heading;
        pmData.speed = gpsData.speed;
        pmData.satellites = (char)gpsData.satellites;
        pmData.positionFix = (char)gpsData.positionFix;        
    }
}

void checkAMData(){
    int i = 0;
    char checksum = 0;
    for (i = 0; i < sizeof(AMData) - 2; i++){
        checksum += ((char *)&amData)[i];
    }
    if (checksum != lastAMDataChecksum && amData.checksum == checksum){
        lastAMDataChecksum = checksum;

        // All commands/actions that need to be run go here
       switch (amData.command){
            case PM_DEBUG_TEST:
                UART1_SendString("Test");
                break;
            case PM_NEW_WAYPOINT:;
                PathData* node = initializePathNode();
                node->altitude = amData.waypoint.altitude;
                node->latitude = amData.waypoint.latitude;
                node->longitude = amData.waypoint.longitude;
                node->radius = amData.waypoint.radius;
                appendPathNode(node);
                UART1_SendString("NODE");
                break;
            case PM_CLEAR_WAYPOINTS:
                clearPathNodes();
            case PM_INSERT_WAYPOINT:
                node = initializePathNode();
                node->altitude = amData.waypoint.altitude;
                node->latitude = amData.waypoint.latitude;
                node->longitude = amData.waypoint.longitude;
                node->radius = amData.waypoint.radius;
                insertPathNode(node,amData.waypoint.previousId,amData.waypoint.nextId);
            case PM_REMOVE_WAYPOINT:
                removePathNode(amData.waypoint.id);
            case PM_SET_CURRENT_WAYPOINT:
               currentIndex = getIndexFromID(amData.waypoint.id);
            case PM_CALIBRATE_ALTIMETER:
                calibrateAltimeter(amData.calibrationHeight);
                break;
            case PM_SET_PATH_GAIN:
                k_gain[PATH] = amData.pathGain;
                break;
            case PM_SET_ORBIT_GAIN:
                k_gain[ORBIT] = amData.orbitGain;
                break;
            default:
                break;
        }
    }
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

