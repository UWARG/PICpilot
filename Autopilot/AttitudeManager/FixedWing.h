/* 
 * File:   FixedWing.h
 * Author: Chris Hajduk / Ian Frosst
 *
 * Created on July 2, 2015, 8:03 PM
 */

#ifndef FIXEDWING_H
#define	FIXEDWING_H

// Header Files
#include "main.h"

// Function Prototypes
void initialization();
void armVehicle(int delayTime);
void dearmVehicle();
void inputMixing(int* channelIn, int* rollRate, int* pitchRate, int* throttle, int* yawRate);
void outputMixing(int* channelOut, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw);
void checkLimits(int* channelOut);
void highLevelControl();
void lowLevelControl();

// Constants
#define MAX_ROLL_PWM (MAX_PWM - 87) //This was - 20
#define MIN_ROLL_PWM (MIN_PWM + 174) //This was + 40
#define MAX_L_TAIL_PWM (MAX_PWM - 109) //This was - 25
#define MIN_L_TAIL_PWM (MIN_PWM + 109) //This was + 25
#define MAX_R_TAIL_PWM (MAX_PWM - 109) //This was - 25
#define MIN_R_TAIL_PWM (MIN_PWM + 109) //This was + 25

#define STALL_SPEED 48.15 //km/h

#define RUDDER_PROPORTION 0.75
#define ELEVATOR_PROPORTION 0.75

#define STANDARD_TAIL 0
#define V_TAIL 1
#define INV_V_TAIL 2

// Set airplane tail type
#define TAIL_TYPE INV_V_TAIL 

#if TAIL_TYPE == STANDARD_TAIL
// Inputs
#define THROTTLE_IN_CHANNEL 1
#define ROLL_IN_CHANNEL 2
#define PITCH_IN_CHANNEL 3
#define YAW_IN_CHANNEL 4
#define FLAP_IN_CHANNEL 5
// Outputs
#define THROTTLE_OUT_CHANNEL 1
#define ROLL_OUT_CHANNEL 2
#define PITCH_OUT_CHANNEL 3
#define YAW_OUT_CHANNEL 4
#define FLAP_OUT_CHANNEL 5

#elif TAIL_TYPE == V_TAIL
// Inputs
#define THROTTLE_IN_CHANNEL 1
#define ROLL_IN_CHANNEL 2
#define L_TAIL_IN_CHANNEL 3
#define R_TAIL_IN_CHANNEL 4
#define FLAP_IN_CHANNEL 5
// Outputs
#define THROTTLE_OUT_CHANNEL 1
#define ROLL_OUT_CHANNEL 2
#define L_TAIL_OUT_CHANNEL 3
#define R_TAIL_OUT_CHANNEL 4
#define FLAP_OUT_CHANNEL 5

#elif TAIL_TYPE == INV_V_TAIL
// Inputs
#define THROTTLE_IN_CHANNEL 1
#define ROLL_IN_CHANNEL 2
#define L_TAIL_IN_CHANNEL 3
#define R_TAIL_IN_CHANNEL 4
#define FLAP_IN_CHANNEL 5
// Outputs
#define THROTTLE_OUT_CHANNEL 1
#define ROLL_OUT_CHANNEL 2
#define L_TAIL_OUT_CHANNEL 3
#define R_TAIL_OUT_CHANNEL 4
#define FLAP_OUT_CHANNEL 5

#endif

#endif	/* FIXEDWING_H */
