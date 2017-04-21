/**
 * @file   Multirotor.h
 * @author Ian Frosst
 * @date February 13, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef MULTIROTOR_H
#define	MULTIROTOR_H

// Header Files
#include "main.h"
#include "PWM.h"
    
// Function Prototypes
void initialization();
void armVehicle(int delayTime);
void dearmVehicle();
void inputMixing(int* channelIn, int* rollRate, int* pitchRate, int* throttle, int* yawRate);
void outputMixing(int* channelOut, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw);
void checkLimits(int* channelOut);
void highLevelControl();
void lowLevelControl();

#define QUAD_X 0
#define QUAD_P 1
#define HEX_X 2 
#define HEX_P 3

// Set multirotor type
#define ROTOR_TYPE QUAD_P

// Inputs
#define THROTTLE_IN_CHANNEL 1
#define ROLL_IN_CHANNEL 2
#define PITCH_IN_CHANNEL 3
#define YAW_IN_CHANNEL 4

// Outputs
#if ROTOR_TYPE == QUAD_X
#define FRONT_LEFT_MOTOR 1
#define FRONT_RIGHT_MOTOR 2
#define BACK_RIGHT_MOTOR 3
#define BACK_LEFT_MOTOR 4

#elif ROTOR_TYPE == QUAD_P
#define FRONT_MOTOR 1
#define RIGHT_MOTOR 2
#define BACK_MOTOR 3
#define LEFT_MOTOR 4

#endif
// TODO: implement channel/motor mixing for other multirotor types

#endif	/* MULTIROTOR_H */

