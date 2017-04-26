/*
 * File:   PathManager.c
 * Author: Chris Hajduk
 *
 * Created on February 4, 2014, 9:13 PM
 */

#include "main.h"
#include "PathManager.h"
#include "../Common/Common.h"
#include "Dubins.h"
#include "MPL3115A2.h"
#include "BatterySensor.h"
#include "airspeedSensor.h"
#include "StartupErrorCodes.h"
#include "../Common/Interfaces/SPI.h"
#include "../Common/Interfaces/InterchipDMA.h"
#include "../Common/Clock/Timer.h"
#include "../Common/Utilities/LED.h"
#include "Peripherals/GPS.h"

#if DEBUG
#include <stdio.h>
#include <stdlib.h>
#include "../Common/Utilities/Logger.h"
#endif


static void checkForFirstGPSLock(); // function prototype for static function at line 679

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
char followPath = 0;
char inHold = 0;

static uint64_t interchip_last_send_time = 0;

static uint8_t led_bright = 0;
static bool going_up = true;

void pathManagerInit(void) {
    initTimer4();
    
    initLED(0);
    
    //Communication with GPS
    initGPS();
    initBatterySensor();
    initAirspeedSensor();

    initSPI(IC_DMA_PORT, DMA_CLOCK_KHZ, SPI_MODE1, SPI_BYTE, SPI_MASTER);
    initInterchip(DMA_CHIP_ID_PATH_MANAGER);

    //Communication with Altimeter
    if (initAltimeter()){
        float initialValue = 0;
        while (initialValue == 0) initialValue = getAltitude();
        calibrateAltimeter(initialValue);
    }
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
#if DEBUG
    char str[20];
    sprintf(str, "PM:%d, AM:%d", sizeof(PMData),sizeof(AMData));
    debug(str);
#endif
}

char buffer[200];

void pathManagerRuntime(void) {
#if DEBUG
//    char str[16];
//    sprintf(&str,"%f",pmData.time);
//    UART1_SendString(&str);
#endif
    requestGPSInfo();

    if (isNewGPSDataAvailable()){
        copyGPSData();
    }
    
    // Update status LED
    if (led_bright == 0) going_up = true;
    else if (led_bright == 255) going_up = false;
    
    if (going_up) led_bright++;
    else led_bright--;
    
    setLEDBrightness(led_bright);

    if (returnHome){
        interchip_send_buffer.pm_data.targetWaypoint = -1;
    } else {
        interchip_send_buffer.pm_data.targetWaypoint = path[currentIndex]->id;
    }

    //Check for new uplink command data
    checkAMData();
    float position[3];
    float heading;
    //Get the position of the plane (in meters)
    getCoordinates(gps_data.longitude,gps_data.latitude,(float*)&position);
    position[2] = gps_data.altitude;
    heading = (float)gps_data.heading;

    if (returnHome || (pathCount - currentIndex < 1 && pathCount >= 0)){
        interchip_send_buffer.pm_data.sp_Heading = lastKnownHeadingHome;
    } else if (followPath && pathCount - currentIndex >= 1 && interchip_send_buffer.pm_data.positionFix > 0) {
        currentIndex = followWaypoints(path[currentIndex], (float*)position, heading, (int*)&interchip_send_buffer.pm_data.sp_Heading);
    }
    if (interchip_send_buffer.pm_data.positionFix >= 1){
        lastKnownHeadingHome = calculateHeadingHome(home, (float*)position, heading);
    }
    
    if (getTimeUs() - interchip_last_send_time >= INTERCHIP_SEND_INTERVAL_US){
        interchip_last_send_time = getTimeUs();
        interchip_send_buffer.pm_data.interchip_error_count = getInterchipErrorCount();
        sendInterchipData();
    }
}

#if DUBINS_PATH
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
//        printf("Recomputing path...\n");
        current_position = (Vector) {
            .x = position[0],
            .y = position[1],
        };
        float angle = deg2rad(90 - heading);
        current_heading = (Vector) {
            .x = cos(angle),
            .y = sin(angle),
        };

        float d = sqrt(pow(target_position.x - current_position.x, 2) + pow(target_position.y - current_position.y, 2));
        if (d < 4*target->radius) {
            return next->index;
        }

        plane = (Line) {
            .initial = current_position,
            .direction = current_heading,
        };
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
#else
char followWaypoints(PathData* currentWaypoint, float* position, float heading, int* sp_Heading){
        float waypointPosition[3];
        getCoordinates(currentWaypoint->longitude, currentWaypoint->latitude, (float*)&waypointPosition);
        waypointPosition[2] = currentWaypoint->altitude;



        PathData* targetWaypoint = currentWaypoint->next;
        float targetCoordinates[3];
        getCoordinates(targetWaypoint->longitude, targetWaypoint->latitude, (float*)&targetCoordinates);
        targetCoordinates[2] = targetWaypoint->altitude;

        PathData* nextWaypoint = targetWaypoint->next;
        float nextCoordinates[3];
        getCoordinates(nextWaypoint->longitude, nextWaypoint->latitude, (float*)&nextCoordinates);
        nextCoordinates[2] = nextWaypoint->altitude;

        float waypointDirection[3];
        float norm = sqrt(pow(targetCoordinates[0] - waypointPosition[0],2) + pow(targetCoordinates[1] - waypointPosition[1],2) + pow(targetCoordinates[2] - waypointPosition[2],2));
        waypointDirection[0] = (targetCoordinates[0] - waypointPosition[0])/norm;
        waypointDirection[1] = (targetCoordinates[1] - waypointPosition[1])/norm;
        waypointDirection[2] = (targetCoordinates[2] - waypointPosition[2])/norm;

        float nextWaypointDirection[3];
        float norm2 = sqrt(pow(nextCoordinates[0] - targetCoordinates[0],2) + pow(nextCoordinates[1] - targetCoordinates[1],2) + pow(nextCoordinates[2] - targetCoordinates[2],2));
        nextWaypointDirection[0] = (nextCoordinates[0] - targetCoordinates[0])/norm2;
        nextWaypointDirection[1] = (nextCoordinates[1] - targetCoordinates[1])/norm2;
        nextWaypointDirection[2] = (nextCoordinates[2] - targetCoordinates[2])/norm2;

        float turningAngle = acos(-deg2rad(waypointDirection[0] * nextWaypointDirection[0] + waypointDirection[1] * nextWaypointDirection[1] + waypointDirection[2] * nextWaypointDirection[2]));

        if (orbitPathStatus == PATH){

            float halfPlane[3];
            halfPlane[0] = targetCoordinates[0] - (targetWaypoint->radius/tan(turningAngle/2)) * waypointDirection[0];
            halfPlane[1] = targetCoordinates[1] - (targetWaypoint->radius/tan(turningAngle/2)) * waypointDirection[1];
            halfPlane[2] = targetCoordinates[2] - (targetWaypoint->radius/tan(turningAngle/2)) * waypointDirection[2];

            float dotProduct = waypointDirection[0] * (position[0] - halfPlane[0]) + waypointDirection[1] * (position[1] - halfPlane[1]) + waypointDirection[2] * (position[2] - halfPlane[2]);
            if (dotProduct > 0){
                orbitPathStatus = ORBIT;
                if (targetWaypoint->type == HOLD_WAYPOINT)
                {
                    inHold = TRUE;
                }
            }

            *sp_Heading = (int)followStraightPath((float*)&waypointDirection, (float*)targetCoordinates, (float*)position, heading);
        }
        else{            
            
            float halfPlane[3];
            halfPlane[0] = targetCoordinates[0] + (targetWaypoint->radius/tan(turningAngle/2)) * nextWaypointDirection[0];
            halfPlane[1] = targetCoordinates[1] + (targetWaypoint->radius/tan(turningAngle/2)) * nextWaypointDirection[1];
            halfPlane[2] = targetCoordinates[2] + (targetWaypoint->radius/tan(turningAngle/2)) * nextWaypointDirection[2];
            
            char turnDirection = waypointDirection[0] * nextWaypointDirection[1] - waypointDirection[1] * nextWaypointDirection[0]>0?1:-1;
            float euclideanWaypointDirection = sqrt(pow(nextWaypointDirection[0] - waypointDirection[0],2) + pow(nextWaypointDirection[1] - waypointDirection[1],2) + pow(nextWaypointDirection[2] - waypointDirection[2],2)) * ((nextWaypointDirection[0] - waypointDirection[0]) < 0?-1:1) * ((nextWaypointDirection[1] - waypointDirection[1]) < 0?-1:1) * ((nextWaypointDirection[2] - waypointDirection[2]) < 0?-1:1);

            float turnCenter[3];
            turnCenter[0] = targetCoordinates[0] + (targetWaypoint->radius/tan(turningAngle/2) * (nextWaypointDirection[0] - waypointDirection[0])/euclideanWaypointDirection);
            turnCenter[1] = targetCoordinates[1] + (targetWaypoint->radius/tan(turningAngle/2) * (nextWaypointDirection[1] - waypointDirection[1])/euclideanWaypointDirection);
            turnCenter[2] = targetCoordinates[2] + (targetWaypoint->radius/tan(turningAngle/2) * (nextWaypointDirection[2] - waypointDirection[2])/euclideanWaypointDirection);
            
            // if target waypoint is a hold waypoint the plane will follow the orbit until the break hold method is called
            if (inHold == TRUE) {
                *sp_Heading = (int)orbitWaypoint((float*) &turnCenter, targetWaypoint->radius, turnDirection, (float*)position, heading); //float OrbitWaypoint (float* center, float radius, char direction, float* position, float heading){
                return currentWaypoint->index;
            }

            float dotProduct = nextWaypointDirection[0] * (position[0] - halfPlane[0]) + nextWaypointDirection[1] * (position[1] - halfPlane[1]) + nextWaypointDirection[2] * (position[2] - halfPlane[2]);
            if (dotProduct > 0){
                orbitPathStatus = PATH;
                return targetWaypoint->index;
            }
            
            //If two waypoints are parallel to each other (no turns)
            if (euclideanWaypointDirection == 0){
                orbitPathStatus = PATH;
                return targetWaypoint->index;
            }

            *sp_Heading = (int)followOrbit((float*) &turnCenter,targetWaypoint->radius, turnDirection, (float*)position, heading);
        }

        return currentWaypoint->index;

}
#endif

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
    waypointPosition[2] = interchip_send_buffer.pm_data.altitude;

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

float OrbitWaypoint (float* center, float radius, char direction, float* position, float heading){
    return (int)followOrbit(center, radius, direction, position, heading);
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
    float calcHeading = 90 - rad2deg(courseAngle - MAX_PATH_APPROACH_ANGLE * 2/PI * atan(k_gain[PATH] * pathError)); //Heading in degrees (magnetic)
    return calcHeading;

}

float maintainAltitude(PathData* cPath){
    float dAltitude = cPath->next->altitude - cPath->altitude;
    float dDistance = getDistance(cPath->longitude, cPath->latitude, cPath->next->longitude, cPath->next->latitude);
    return getDistance(cPath->next->longitude, cPath->next->latitude, gps_data.longitude, gps_data.latitude)/dDistance * dAltitude + cPath->altitude;
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

    //Check to make sure it is not a duplicate of the previous node
    if (previousNode->latitude == node->latitude && previousNode->longitude == node->longitude){
        return -1;
    }

    if (node->index){
        path[currentBufferIndex] = node;
        node->index = currentBufferIndex++;
    }

    pathCount++;
    //Update previous node
    previousNode->next = node;

    return node->id;
}

unsigned int updatePathNode(PathData* node, unsigned int ID){
    unsigned int nodeIndex = getIndexFromID(ID);
    if (nodeIndex == -1){
        return -1;
    }
    PathData* currentNode = path[nodeIndex];
    PathData* previousNode = currentNode->previous;
    PathData* nextNode = currentNode->next;

    //Reassign pointers to new node
    node->next = nextNode;
    node->previous = previousNode;
    previousNode->next = node;
    nextNode->previous = node;

    //Destroy old node
    destroyPathNode(currentNode);
    currentNode = 0;

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
    interchip_send_buffer.pm_data.time = gps_data.utc_time;
    interchip_send_buffer.pm_data.longitude = gps_data.longitude;
    interchip_send_buffer.pm_data.latitude = gps_data.latitude;
    interchip_send_buffer.pm_data.heading = gps_data.heading;
    interchip_send_buffer.pm_data.speed = gps_data.ground_speed;
    interchip_send_buffer.pm_data.satellites = (char)gps_data.num_satellites;
    interchip_send_buffer.pm_data.positionFix = (char)gps_data.fix_status;
    
    checkForFirstGPSLock();
    
    interchip_send_buffer.pm_data.batteryLevel1 = getMainBatteryLevel();
    interchip_send_buffer.pm_data.batteryLevel2 = getExtBatteryLevel();
    interchip_send_buffer.pm_data.airspeed = getCurrentAirspeed();
    interchip_send_buffer.pm_data.altitude = getAltitude(); //want to get altitude regardless of if there is new GPS data
    interchip_send_buffer.pm_data.pmOrbitGain = k_gain[ORBIT];
    interchip_send_buffer.pm_data.pmPathGain = k_gain[PATH];
    interchip_send_buffer.pm_data.waypointCount = pathCount;
    interchip_send_buffer.pm_data.waypointChecksum = getWaypointChecksum();
    interchip_send_buffer.pm_data.pathFollowing = followPath;
}

static void checkForFirstGPSLock(){
	static char gpsLockFlag = 1;
    
    if (gpsLockFlag){
        unsigned int error_codes = getErrorCodes();
        
        if((error_codes & STARTUP_ERROR_BROWN_OUT_RESET) || (error_codes & STARTUP_ERROR_POWER_ON_RESET)){
		home.latitude = gps_data.latitude;
		home.longitude = gps_data.longitude;
		home.altitude = gps_data.altitude;
			
		gpsLockFlag = 0;
        }
    }
}

uint8_t last_amdata_checksum = 0;

void checkAMData(){
    if (!newInterchipData()){
        return;
    }
    switch (interchip_receive_buffer.am_data.command){
            case PM_DEBUG_TEST:
//                UART1_SendString("Test");
                break;
            case PM_NEW_WAYPOINT:;
                PathData* node = initializePathNode();
                node->altitude = interchip_receive_buffer.am_data.waypoint.altitude;
                node->latitude = interchip_receive_buffer.am_data.waypoint.latitude;
                node->longitude = interchip_receive_buffer.am_data.waypoint.longitude;
                node->radius = interchip_receive_buffer.am_data.waypoint.radius;
                node->type = interchip_receive_buffer.am_data.waypoint.type;
                appendPathNode(node);
//                    debug("new");
//                    char str[20];
//                    sprintf(str, "Lat: %f, Lon: %f, count: %d", node->latitude, node->longitude, pathCount);
//                    debug(str);

                break;
            case PM_CLEAR_WAYPOINTS:
                clearPathNodes();
                break;
            case PM_INSERT_WAYPOINT:
                node = initializePathNode();
                node->altitude = interchip_receive_buffer.am_data.waypoint.altitude;
                node->latitude = interchip_receive_buffer.am_data.waypoint.latitude;
                node->longitude = interchip_receive_buffer.am_data.waypoint.longitude;
                node->radius = interchip_receive_buffer.am_data.waypoint.radius;
                node->type = interchip_receive_buffer.am_data.waypoint.type;
                insertPathNode(node,interchip_receive_buffer.am_data.waypoint.previousId,interchip_receive_buffer.am_data.waypoint.nextId);
                break;
            case PM_UPDATE_WAYPOINT:
                node = initializePathNode();
                node->altitude = interchip_receive_buffer.am_data.waypoint.altitude;
                node->latitude = interchip_receive_buffer.am_data.waypoint.latitude;
                node->longitude = interchip_receive_buffer.am_data.waypoint.longitude;
                node->radius = interchip_receive_buffer.am_data.waypoint.radius;
                node->type = interchip_receive_buffer.am_data.waypoint.type;
                updatePathNode(node,interchip_receive_buffer.am_data.waypoint.id);
               break;
            case PM_REMOVE_WAYPOINT:
                removePathNode(interchip_receive_buffer.am_data.waypoint.id);
                break;
            case PM_SET_TARGET_WAYPOINT:
//                    node = initializePathNode();
//                    node->altitude = gpsData.altitude;
//                    node->latitude = gpsData.latitude;
//                    node->longitude = gpsData.longitude;
//                    node->radius = 1; //Arbitrary value
//                    node->type = DEFAULT_WAYPOINT;
                if (path[getIndexFromID(interchip_receive_buffer.am_data.waypoint.id)] && path[getIndexFromID(interchip_receive_buffer.am_data.waypoint.id)]->previous){
//                        insertPathNode(node,path[getIndexFromID(amData.waypoint.id)]->previous->id,amData.waypoint.id);
                    currentIndex = getIndexFromID(interchip_receive_buffer.am_data.waypoint.id);
                }
                returnHome = 0;
                break;
            case PM_SET_RETURN_HOME_COORDINATES:
                home.altitude = interchip_receive_buffer.am_data.waypoint.altitude;
                home.latitude = interchip_receive_buffer.am_data.waypoint.latitude;
                home.longitude = interchip_receive_buffer.am_data.waypoint.longitude;
                home.radius = 1;
                home.id = -1;
                home.type = DEFAULT_WAYPOINT;
                break;
            case PM_RETURN_HOME:
                returnHome = 1;
                break;
            case PM_CANCEL_RETURN_HOME:
                returnHome = 0;
                break;
           case PM_FOLLOW_PATH:
                followPath = interchip_receive_buffer.am_data.followPath;
                break;
           case PM_EXIT_HOLD_ORBIT:
               inHold = FALSE;
               break;
            case PM_CALIBRATE_ALTIMETER:
                calibrateAltimeter(interchip_receive_buffer.am_data.calibrationHeight);
                break;
            case PM_SET_PATH_GAIN:
                k_gain[PATH] = interchip_receive_buffer.am_data.pathGain;
                break;
            case PM_SET_ORBIT_GAIN:
                k_gain[ORBIT] = interchip_receive_buffer.am_data.orbitGain;
                break;
            default:
                break;
        }
}

float getWaypointChecksum(void){
    unsigned int i = 0;
    float checksum = 0;
    for (i = 0; i < pathCount; i++){
        checksum += path[i]->latitude + path[i]->longitude + path[i]->altitude + path[i]->radius + (float)path[i]->type;
    }
    return checksum;
}

