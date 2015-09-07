/* 
 * File:   Anaconda.h
 * Author: Chris Hajduk
 *
 * Created on July 2, 2015, 8:03 PM
 */

#ifndef ANACONDA_H
#define	ANACONDA_H

#if ANACONDA_VEHICLE

#ifdef	__cplusplus
extern "C" {
#endif

// Header Files
#include "main.h"
#include "AttitudeManager.h"

// Function Prototypes
void inputMixing(int* channels, int* rollRate, int* pitchRate, int* throttle, int* yawRate);
void outputMixing(int* channels, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw);
void checkLimits(int* channels);

// Constants
#define MAX_ROLL_PWM (MAX_PWM - 87) //This was - 20
#define MIN_ROLL_PWM (MIN_PWM + 174) //This was + 40
#define MAX_PITCH_PWM (MAX_PWM - 109) //This was - 25
#define MIN_PITCH_PWM (MIN_PWM + 109) //This was + 25
#define MAX_YAW_PWM (MAX_PWM - 109) //This was - 25
#define MIN_YAW_PWM (MIN_PWM + 109) //This was + 25

#define TAIL_TYPE 2 // Standard (0) or V-Tail (1) or Inverse V-Tail (2)
#define STANDARD_TAIL 0
#define V_TAIL 1
#define INV_V_TAIL 2

#define RUDDER_PROPORTION 0.75
#define ELEVATOR_PROPORTION 0.75

#ifdef	__cplusplus
}
#endif

#endif

#endif	/* ANACONDA_H */

