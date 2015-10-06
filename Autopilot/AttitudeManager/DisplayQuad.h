/* 
 * File:   DisplayQuad.h
 * Author: Chris Hajduk
 *
 * Created on September 6, 2015, 1:02 PM
 */

#ifndef DISPLAYQUAD_H
#define	DISPLAYQUAD_H

#ifdef	__cplusplus
extern "C" {
#endif

// Header Files
#include "main.h"

// Function Prototypes
void initialization();
void armVehicle(int delayTime);
void dearmVehicle();
void inputMixing(int* channels, int* rollRate, int* pitchRate, int* throttle, int* yawRate);
void outputMixing(int* channels, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw);
void checkLimits(int* channels);
void startArm();
void stopArm();

// Constants
#define MAX_ROLL_PWM (MAX_PWM - 87) //This was - 20
#define MIN_ROLL_PWM (MIN_PWM + 174) //This was + 40
#define MAX_PITCH_PWM (MAX_PWM - 109) //This was - 25
#define MIN_PITCH_PWM (MIN_PWM + 109) //This was + 25
#define MAX_YAW_PWM (MAX_PWM - 109) //This was - 25
#define MIN_YAW_PWM (MIN_PWM + 109) //This was + 25

#ifdef	__cplusplus
}
#endif

#endif	/* DISPLAYQUAD_H */

