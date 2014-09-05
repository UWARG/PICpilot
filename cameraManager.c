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
float pictureDistance = 40; //In meters
unsigned int lastSignal = LOWER_PWM;
unsigned int triggerSignal = 600;
unsigned int gimbleOffset = MIDDLE_PWM - 42;
int rollLimit = 30;
char overrideTrigger = 0;
char resting = 1;

unsigned int cameraPollingRuntime(long double latitude, long double longitude, long int time, unsigned int* pictureCount, int rollAngle, int pitchAngle){
    distance = getDistance(latitude, longitude, lastLatitude, lastLongitude);
//    char str[16];
//    sprintf(&str, "%f", distance);
//    UART1_SendString(&str);

    if (time - cameraTimerCount > 1350){
        cameraTimerCount = time;
        if (((((distance >= pictureDistance || distance <= -pictureDistance) && (rollAngle <= rollLimit || rollAngle >= -rollLimit) && (pitchAngle <= rollLimit || pitchAngle >= -rollLimit)))|| overrideTrigger) && resting){// && pitch <= 20 && pitch >= -20 && roll >= -20 && roll <= 20){
            lastLongitude = longitude;
            lastLatitude = latitude;
            lastSignal = LOWER_PWM;
            overrideTrigger = 0;
            (*pictureCount)++;
            resting = 0;
        }
        else{
            resting = 1;
            lastSignal = triggerSignal;
        }
    }
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
    if (rollAngle > RIGHT_GIMBLE_MOTION_LIMIT){
        rollAngle = RIGHT_GIMBLE_MOTION_LIMIT;
    }
    if (rollAngle < -LEFT_GIMBLE_MOTION_LIMIT){
        rollAngle = -LEFT_GIMBLE_MOTION_LIMIT;
    }

    return gimbleOffset + GIMBLE_PWM_RANGE/GIMBLE_MOTION_RANGE * rollAngle;
}

void setGimbleOffset(unsigned int pwmSignal){
    gimbleOffset = pwmSignal + MIDDLE_PWM;
}