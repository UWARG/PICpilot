/*
 * File:   cameraManager.c
 * Author: Chris Hajduk
 *
 * Created on March 8, 2014, 5:22 PM
 */

//Header Files
#include "cameraManager.h"
#include "main.h"
#include "PathManager.h" //To get Earth Radius Constant
#include "OutputCompare.h"



float distance = 0;
long double lastLongitude = 0;
long double lastLatitude = 0;

unsigned int cameraPollingRuntime(long double latitude, long double longitude, unsigned int cSwitch){
    distance = getDistance(latitude, longitude, lastLatitude, lastLongitude);
    if (distance >= 25 && cSwitch < 600){// && pitch <= 20 && pitch >= -20 && roll >= -20 && roll <= 20){
        lastLongitude = longitude;
        lastLatitude = latitude;
        return UPPER_PWM;
    }
    else
        return 0;
}
unsigned int cameraGimbleStabilization(float rollAngle){
    if (rollAngle > GIMBLE_MOTION_LIMIT){
        rollAngle = GIMBLE_MOTION_LIMIT;
    }
    if (rollAngle < -GIMBLE_MOTION_LIMIT){
        rollAngle = -GIMBLE_MOTION_LIMIT;
    }

    return GIMBLE_PWM_OFFSET + GIMBLE_PWM_RANGE/GIMBLE_MOTION_RANGE * rollAngle;
}