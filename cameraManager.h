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
#define GIMBLE_PWM_OFFSET MIDDLE_PWM + 45
#define GIMBLE_MOTION_RANGE 45 //Range one way
#define GIMBLE_MOTION_LIMIT 17.5 //Range One way

//TODO:Add a description here
unsigned int cameraPollingRuntime(long double latitude, long double longitude, unsigned int cSwitch);
unsigned int cameraGimbleStabilization(float imu_RollAngle);

#endif	/* CAMERAMANAGER_H */

