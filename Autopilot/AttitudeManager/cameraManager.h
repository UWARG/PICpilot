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
#define GIMBAL_PWM_RANGE HALF_PWM_RANGE //Range one way (IE. Positive or Negative range, not both)
#define GIMBAL_MOTION_RANGE 30 //Range one way
#define LEFT_GIMBAL_MOTION_LIMIT 30 //Range Left way
#define RIGHT_GIMBAL_MOTION_LIMIT 30 //Range Right way
#define GOPRO_GIMBAL_MOTION_RANGE 30 //Range one way
#define LEFT_GIMBAL_GOPRO_MOTION_LIMIT 30 //Range Left way
#define RIGHT_GIMBAL_GOPRO_MOTION_LIMIT 30 //Range Right way
#define VERTICAL_MOTION_RANGE 30 //Range one way
#define UP_MOTION_LIMIT 30 //Range up
#define DOWN_MOTION_LIMIT 30 //Range down

//TODO:Add a description here
unsigned int cameraPollingRuntime(long double latitude, long double longitude, long int time, unsigned int* cameraCounter, int rollAngle, int pitchAngle);
void triggerCamera(int pwmSignal);
void setTriggerDistance(float distance);
int cameraGimbalStabilization(float imu_RollAngle);
void setGimbalOffset(int pwmSignal);
int goProGimbalStabilization(float rollAngle);
void setGoProGimbalOffset(int pwmSignal);
int goProVerticalstabilization(float pitchAngle);
void setVerticalOffset(int pwmSignal);
void lockGoPro(int lock);

#endif	/* CAMERAMANAGER_H */