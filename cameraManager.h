/* 
 * File:   cameraManager.h
 * Author: Chris Hajduk
 *
 * Created on March 8, 2014, 5:22 PM
 */

#ifndef CAMERAMANAGER_H
#define	CAMERAMANAGER_H

#include "OutputCompare.h"

//Constants
#define GIMBLE_PWM_RANGE SP_RANGE //Range one way (IE. Positive or Negative range, not both)
#define GIMBLE_MOTION_RANGE 40.0 //Range one way
#define LEFT_GIMBLE_MOTION_LIMIT 13 //Range Left way
#define RIGHT_GIMBLE_MOTION_LIMIT 20 //Range Right way

//TODO:Add a description here
unsigned int cameraPollingRuntime(long double latitude, long double longitude, long int time, unsigned int* cameraCounter, int rollAngle, int pitchAngle);
void triggerCamera(unsigned int pwmSignal);
void setTriggerDistance(float distance);
unsigned int cameraGimbleStabilization(float imu_RollAngle);
void setGimbleOffset(unsigned int pwmSignal);

#endif	/* CAMERAMANAGER_H */

