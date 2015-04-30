/*
 * File:   PathManager.c
 * Author: Chris Hajduk
 *
 * Created on February 4, 2014, 9:13 PM
 */

#include "main.h"
#include "PathManager.h"
#include "Dubins.h"
#include "MPL3115A2.h"
#include "voltageSensor.h"
#include "UART2.h"
#include "NMEAparser.h"

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

PathData home;

float k_gain[2] = {0.01, 1};

unsigned int currentBufferIndex = 0; //Last index that was filled
unsigned int currentNodeID = 0; //Last ID that was used
unsigned int currentIndex = 0; //Current Index that is being followed

char orbitPathStatus = PATH;

char lastAMDataChecksum = 0;

PathData* path[PATH_BUFFER_SIZE];
char pathStatus[PATH_BUFFER_SIZE];
char pathCount = 0;

int lastKnownHeadingHome = 10;
char returnHome = 0;

void pathManagerInit(void) {
#if DEBUG
    InitUART1();
#endif



    //Communication with GPS
//    InitUART2();
#if GPS_OLD
    init_SPI2();
    init_DMA2();
//    // Hack to power altimeter from UART on PM
    TRISFbits.TRISF5 = 0;
    PORTFbits.RF5 = 1;
#else
    InitUART2();
#endif
    initBatterySensor();

    //Interchip Communication
#if !ATTITUDE_MANAGER
    //Initialize Interchip Interrupts for Use in DMA Reset
    //Set opposite Input / Output Configuration on the AttitudeManager
    TRISAbits.TRISA12 = 0;  //Init RA12 as Output (0)
    TRISAbits.TRISA13 = 0;  //Init RA13 as Output (0)

    TRISBbits.TRISB4 = 1;   //Init RB4 as Input (1)
    TRISBbits.TRISB5 = 1;   //Init RB5 as Input (1)

    INTERCOM_3 = 0;    //Set RA12 to Output a Value of 0
    INTERCOM_4 = 0;    //Set RA13 to Output a Value of 0

    init_SPI1();
    init_DMA0();
    init_DMA1();
    DMA1REQbits.FORCE = 1;
    while (DMA1REQbits.FORCE == 1);

    //Communication with Altimeter
    if (initAltimeter()){
        float initialValue = 0;
        while (initialValue == 0) initialValue = getAltitude();
        calibrateAltimeter(initialValue);
    }
#endif

    //Initialize Home Location
    home.altitude = 400;
    home.latitude = RELATIVE_LATITUDE;
    home.longitude = RELATIVE_LONGITUDE;
    home.radius = 1;
    home.id = -1;

    //Initialize first path nodes
//    PathData* node = initializePathNode();
//    node->altitude = 10;
//    node->latitude = 43.473662;
//    node->longitude = -80.540019;
//    node->radius = 5;
//    appendPathNode(node);
//    node = initializePathNode();
//    node->altitude = 10;
//    node->latitude = 43.473479;
//    node->longitude = -80.540601;
//    node->radius = 5;
//    appendPathNode(node);
//    node = initializePathNode();
//    node->altitude = 10;
//    node->latitude = 43.473718;
//    node->longitude = -80.540837;
//    node->radius = 5;
//    appendPathNode(node);
//    node = initializePathNode();
//    node->altitude = 10;
//    node->latitude = 43.473946;
//    node->longitude = -80.540261;
//    node->radius = 5;
//    appendPathNode(node);
//    node = initializePathNode();
//    node->altitude = 10;
//    node->latitude = 43.473685;
//    node->longitude = -80.540073;
//    node->radius = 5;
//    appendPathNode(node);
}

void pathManagerRuntime(void) {
#if DEBUG
//        char str[16];
//        sprintf(&str,"%f",pmData.time);
//        UART1_SendString(&str);
#endif
    //Get GPS data
#if !GPS_OLD
    assembleNEMAMessage();
#endif

    copyGPSData();

    if (returnHome){
        pmData.targetWaypoint = -1;
    } else {
        pmData.targetWaypoint = path[currentIndex]->id;
    }

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

    if (returnHome || (pathCount - currentIndex < 1 && pathCount >= 0)){
        printf("Heading home...\n");
        pmData.sp_Heading = lastKnownHeadingHome;
    } else if (pathCount - currentIndex >= 1 && pmData.positionFix > 0) {
        currentIndex = followWaypoints(path[currentIndex], (float*)&position, heading, (int*)&pmData.sp_Heading);
    }
    if (pmData.positionFix > 0){
        lastKnownHeadingHome = calculateHeadingHome(home, (float*)&position, heading);
    }

    pmData.checksum = generatePMDataChecksum();
}

char followWaypoints(PathData* current, float* position, float heading, int* setpoint) {
    static char recompute = 1;
    static float radius = 5; // get from first waypoint?
    static Vector current_position, target_position, next_position;

    PathData* target = current;
    PathData* next;
    if (target->next) {
        next = target->next;
    } else {
        next = &home;
    }

    getCoordinates(target->longitude, target->latitude, (float*)&target_position);
    getCoordinates(next->longitude, next->latitude, (float*)&next_position);

    static Vector current_heading, target_heading;
    get_direction(&target_position, &next_position, &target_heading);

    static Circle close, far;
    static Line tangent, plane;

    static DubinsPath progress = DUBINS_PATH_C1;

    if (recompute) {
        printf("Recomputing path...\n");
        current_position = (Vector) {
            .x = position[0],
            .y = position[1],
        };
        float angle = deg2rad(90 - heading);
        current_heading = (Vector) {
            .x = cos(angle),
            .y = sin(angle),
        };
        plane = (Line) {
            .initial = current_position,
            .direction = current_heading,
        };

        // can figure out both from this
        // always will take same pair
        if (belongs_to_half_plane(&plane, &next_position)) {
            close.center = (Vector) {
                .x = current_position.x + current->radius*current_heading.y,
                .y = current_position.y - current->radius*current_heading.x,
            };
            far.center = (Vector) {
                .x = target_position.x - target->radius*target_heading.y,
                .y = target_position.y + target->radius*target_heading.x,
            };
        } else {
            close.center = (Vector) {
                .x = current_position.x - current->radius*current_heading.y,
                .y = current_position.y + current->radius*current_heading.x,
            };
            far.center = (Vector) {
                .x = target_position.x + target->radius*target_heading.y,
                .y = target_position.y - target->radius*target_heading.x,
            };
        }
        close.radius = target->radius;
        far.radius = target->radius;

        Line tangents[2];
        get_tangents(&close, &far, tangents);

        float d1 = sqrt(pow(tangents[0].initial.x - current_position.x, 2) + pow(tangents[0].initial.y - current_position.y, 2));
        float d2 = sqrt(pow(tangents[1].initial.x - current_position.x, 2) + pow(tangents[1].initial.y - current_position.y, 2));
        tangent = d1 < d2 ? tangents[0] : tangents[1];

        progress = DUBINS_PATH_C1;
        radius = target->radius;
        recompute = 0;
    }

    plane = (Line) {
        .initial = tangent.initial,
        .direction = (Vector) {
            .x = -tangent.direction.y,
            .y = tangent.direction.x,
        },
    };
    // before crossing first tangent point
    if (belongs_to_half_plane(&plane, (Vector *)position)) {
        char direction = current_heading.x*tangent.direction.y - current_heading.y*tangent.direction.x > 0 ? 1 : -1;
        *setpoint = (int)followOrbit((float *)&close.center, close.radius, direction, position, heading);
    } else {
        plane.initial = (Vector) {
            .x = tangent.initial.x + tangent.direction.x,
            .y = tangent.initial.y + tangent.direction.y,
        };
        // before crossing second tangent point
        if (belongs_to_half_plane(&plane, (Vector *)position)) {
            *setpoint = (int)followStraightPath((float*)&tangent.direction, (float*)&plane.initial, position, heading);
        } else {
            plane = (Line) {
                .initial = target_position,
                .direction = (Vector) {
                    .x = -target_heading.y,
                    .y = target_heading.x,
                },
            };
            // before crossing target waypoint
            if (belongs_to_half_plane(&plane, (Vector *)position)) {
                char direction = tangent.direction.x*target_heading.y - tangent.direction.y*target_heading.x > 0 ? 1 : -1;
                *setpoint = (int)followOrbit((float *)&far.center, far.radius, direction, position, heading);
            } else {
                recompute = 1;
                return next->index;
            }
        }
    }
    return current->index;
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

        return (int)followStraightPath((float*)&waypointDirection, (float*)targetCoordinates, (float*)position, heading);
}

int followLastLineSegment(PathData* currentWaypoint, float* position, float heading){
    float waypointPosition[3];
    waypointPosition[0] = position[0];
    waypointPosition[1] = position[1];
    waypointPosition[2] = pmData.altitude;

    PathData* targetWaypoint = currentWaypoint;
    float targetCoordinates[3];
    getCoordinates(targetWaypoint->longitude, targetWaypoint->latitude, (float*)&targetCoordinates);
    targetCoordinates[2] = targetWaypoint->altitude;

    float waypointDirection[3];
    float norm = sqrt(pow(targetCoordinates[0] - waypointPosition[0],2) + pow(targetCoordinates[1] - waypointPosition[1],2) + pow(targetCoordinates[2] - waypointPosition[2],2));
    waypointDirection[0] = (targetCoordinates[0] - waypointPosition[0])/norm;
    waypointDirection[1] = (targetCoordinates[1] - waypointPosition[1])/norm;
    waypointDirection[2] = (targetCoordinates[2] - waypointPosition[2])/norm;

    float dotProduct = waypointDirection[0] * (position[0] - targetCoordinates[0]) + waypointDirection[1] * (position[1] - targetCoordinates[1]) + waypointDirection[2] * (position[2] - targetCoordinates[2]);
    if (dotProduct > 0){
        returnHome = 1;
    }

    return (int)followStraightPath((float*)&waypointDirection, (float*)targetCoordinates, (float*)position, heading);
}

// direction: ccw = 1, cw = -1
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
float followStraightPath(float* waypointDirection, float* targetWaypoint, float* position, float heading){ //Heading in degrees (magnetic)
    heading = deg2rad(90 - heading);//90 - heading = magnetic heading to cartesian heading
    float courseAngle = atan2(waypointDirection[1], waypointDirection[0]); // (y,x) format
    while (courseAngle - heading < -PI){
        courseAngle += 2 * PI;
    }
    while (courseAngle - heading > PI){
        courseAngle -= 2 * PI;
    }

    float pathError = -sin(courseAngle) * (position[0] - targetWaypoint[0]) + cos(courseAngle) * (position[1] - targetWaypoint[1]);

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

int calculateHeadingHome(PathData home, float* position, float heading){
        float waypointPosition[3];
        getCoordinates(position[0], position[1], (float*)&waypointPosition);
        waypointPosition[2] = position[2];

        float targetCoordinates[3];
        getCoordinates(home.longitude, home.latitude, (float*)&targetCoordinates);
        targetCoordinates[2] = home.altitude;


        float waypointDirection[3];
        float norm = sqrt(pow(targetCoordinates[0] - waypointPosition[0],2) + pow(targetCoordinates[1] - waypointPosition[1],2) + pow(targetCoordinates[2] - waypointPosition[2],2));
        waypointDirection[0] = (targetCoordinates[0] - waypointPosition[0])/norm;
        waypointDirection[1] = (targetCoordinates[1] - waypointPosition[1])/norm;
        waypointDirection[2] = (targetCoordinates[2] - waypointPosition[2])/norm;

        heading = deg2rad(90 - heading);//90 - heading = magnetic heading to cartesian heading
        float courseAngle = atan2(waypointDirection[1], waypointDirection[0]);

        //Don't use follow straight path in emergency situations (the gains will give you a heading that will converge over time, but not instantaneously)
        return (int)(90 - rad2deg(courseAngle));
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
    if (node->index){
        path[currentBufferIndex] = node;
        node->index = currentBufferIndex++;
    }

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
    pathCount--;
    return ID;
}
void clearPathNodes(void){
    int i = 0;
    for (i = 0; i < PATH_BUFFER_SIZE; i++){
        if (path[i]){
            destroyPathNode(path[i]);
        }
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
    //Setup the node object first if it hasn't been initialized properly
    if (node->index == 0){
        path[currentBufferIndex] = node;
        node->index = currentBufferIndex++;
    }
    node->next = nextNode;
    node->previous = previousNode;

    //Update previous and next nodes
    nextNode->previous = node;
    previousNode->next = node;

    pathCount++;
    return node->id;
}

void copyGPSData(){
    if (newGPSDataAvailable){
        newGPSDataAvailable = 0;
        pmData.time = gpsData.time;
        pmData.longitude = gpsData.longitude;
        pmData.latitude = gpsData.latitude;
        pmData.heading = gpsData.heading;
        pmData.speed = gpsData.speed;
        pmData.satellites = (char)gpsData.satellites;
        pmData.positionFix = (char)gpsData.positionFix;
        pmData.batteryLevel = getCurrentPercent();
    }
    pmData.altitude = getAltitude(); //gpsData.altitude; //want to get altitude regardless of if there is new GPS data
    pmData.checksum = generatePMDataChecksum();
}


// TODO: make me a real checksum!
char generatePMDataChecksum(void) {
    return 0xAA;
}

void checkAMData(){
    char checksum = 0xAB;
    if (amData.checksum == checksum){
       // All commands/actions that need to be run go here
       switch (amData.command){
            case PM_DEBUG_TEST:
//                UART1_SendString("Test");
                break;
            case PM_NEW_WAYPOINT:;
                PathData* node = initializePathNode();
                node->altitude = amData.waypoint.altitude;
                node->latitude = amData.waypoint.latitude;
                node->longitude = amData.waypoint.longitude;
                node->radius = amData.waypoint.radius;
                appendPathNode(node);
                break;
            case PM_CLEAR_WAYPOINTS:
                clearPathNodes();
                break;
            case PM_INSERT_WAYPOINT:
                node = initializePathNode();
                node->altitude = amData.waypoint.altitude;
                node->latitude = amData.waypoint.latitude;
                node->longitude = amData.waypoint.longitude;
                node->radius = amData.waypoint.radius;
                insertPathNode(node,amData.waypoint.previousId,amData.waypoint.nextId);
                break;
            case PM_REMOVE_WAYPOINT:
                removePathNode(amData.waypoint.id);
                break;
            case PM_SET_TARGET_WAYPOINT:
                node = initializePathNode();
                node->altitude = gpsData.altitude;
                node->latitude = gpsData.latitude;
                node->longitude = gpsData.longitude;
                node->radius = 1; //Arbitrary value
                if (path[getIndexFromID(amData.waypoint.id)] && path[getIndexFromID(amData.waypoint.id)]->previous){
                    insertPathNode(node,path[getIndexFromID(amData.waypoint.id)]->previous->id,amData.waypoint.id);
                    currentIndex = node->index;
                }
                returnHome = 0;

                break;
            case PM_SET_RETURN_HOME_COORDINATES:
                home.altitude = amData.waypoint.altitude;
                home.latitude = amData.waypoint.latitude;
                home.longitude = amData.waypoint.longitude;
                home.radius = 1;
                home.id = -1;
                break;
            case PM_RETURN_HOME:
                returnHome = 1;
                break;
            case PM_CANCEL_RETURN_HOME:
                returnHome = 0;
                break;

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

char getWaypointChecksum(void){
    unsigned int i = 0;
    char checksum = 0;
    for (i = 0; i < pathCount; i++){
        checksum ^= (char)(path[i]->latitude + path[i]->longitude + path[i]->altitude + path[i]->radius) & 0xFF;
    }
    return checksum;
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

