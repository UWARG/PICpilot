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
long int cameraTimerCount = 0;
float pictureDistance = 20; //In meters
unsigned int lastSignal = LOWER_PWM;
unsigned int triggerSignal = 600;
char overrideTrigger = 0;

unsigned int cameraPollingRuntime(long double latitude, long double longitude, long int time){
    distance = getDistance(latitude, longitude, lastLatitude, lastLongitude);
//    char str[16];
//    sprintf(&str, "%f", distance);
//    UART1_SendString(&str);

    if (time - cameraTimerCount > 1000){
        cameraTimerCount = time;
        if (distance >= pictureDistance || distance <= -pictureDistance || overrideTrigger){// && pitch <= 20 && pitch >= -20 && roll >= -20 && roll <= 20){
            lastLongitude = longitude;
            lastLatitude = latitude;
            lastSignal = LOWER_PWM;
            overrideTrigger = 0;
//            UART1_SendString("PIC");
        }
        else
            lastSignal = triggerSignal;
    }
//    lastSignal = 10000;
//    sprintf(&str, "%d", lastSignal);
//    UART1_SendString(&str);
    return lastSignal;
}

void triggerCamera(unsigned int pwmSignal){
    overrideTrigger = 1;
    triggerSignal = pwmSignal;
}

void setTriggerDistance(float distance){
    pictureDistance = distance;
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