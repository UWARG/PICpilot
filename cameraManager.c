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
#include "PWM.h"



float distance = 0;
long double lastLongitude = 0;
long double lastLatitude = 0;
long int cameraTimerCount = 0;
float pictureDistance = 40; //In meters
unsigned int lastSignal = LOWER_PWM;
unsigned int triggerSignal = 600;
unsigned int gimbalOffset = 240;//was MIDDLE_PWM -42; 300 at current config gets the camera pointed towards the ground
unsigned int goProgimbalOffset = 325;//gets the gimbal arm horizontal to the ground at current configuration
unsigned int verticalOffset = -100;//at current calibration of servos, -100 gets us looking face down; 400 gives 20 degrees from straight down in case we want to look forward ish
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

//camera gimbal (roll; no pitch needed)
int cameraGimbalStabilization(float rollAngle){
    if (rollAngle > LEFT_GIMBAL_MOTION_LIMIT){//makes sure servo doesn't go past set limits
        rollAngle = LEFT_GIMBAL_MOTION_LIMIT;
    }
    if (rollAngle < -RIGHT_GIMBAL_MOTION_LIMIT){
        rollAngle = -RIGHT_GIMBAL_MOTION_LIMIT;
    }

    return gimbalOffset - 675/GIMBAL_MOTION_RANGE * rollAngle;//1024 is the same as GIMBAL_PWM_RANGE; 675 for some reason gives better stabiization with motion range at 30
    //compared to 1024 with range at 60. This is being used for all of the stabilization codes (camera, GoPro, vertical)
    
}

void setGimbalOffset(int pwmSignal){
    gimbalOffset =  pwmSignal; // + MIDDLE_PWM;
}

//GoPro gimbal (roll)
int goProGimbalStabilization(float rollAngle){
    if (rollAngle > LEFT_GIMBAL_GOPRO_MOTION_LIMIT){
        rollAngle = LEFT_GIMBAL_GOPRO_MOTION_LIMIT;
    }
    if (rollAngle < - RIGHT_GIMBAL_GOPRO_MOTION_LIMIT){
        rollAngle = - RIGHT_GIMBAL_GOPRO_MOTION_LIMIT;
    }

    return goProgimbalOffset + 675/GOPRO_GIMBAL_MOTION_RANGE * rollAngle;
}// there is a plus because the servo is facing the opposite way in terms of the camera gimbal servo

void setGoProGimbalOffset(int pwmSignal){
    goProgimbalOffset =  pwmSignal; // + MIDDLE_PWM;
}

//GoPro gimbal (pitch)
int goProVerticalstabilization(float pitchAngle){
    if (pitchAngle > UP_MOTION_LIMIT){
        pitchAngle = UP_MOTION_LIMIT;
    }
    if (pitchAngle < - DOWN_MOTION_LIMIT){
        pitchAngle = - DOWN_MOTION_LIMIT;
    }

    return verticalOffset - 675/VERTICAL_MOTION_RANGE * pitchAngle;
}

void setVerticalOffset(int pwmSignal){
    verticalOffset =  pwmSignal; // + MIDDLE_PWM;
}