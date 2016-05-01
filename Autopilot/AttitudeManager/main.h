/* 
 * File:   main.h
 * Author: Chris Hajduk
 *
 * Created on October 15, 2013, 8:14 PM
 *
 * This file contains all GLOBAL constants needed for the code.
 * Avoiding adding stuff here unless you really need to.
 *
 */

//Include Libraries
#include "../Common/Common.h"
#include "timer.h"
#include "delay.h"

//Turns debug mode on or off. Usually contains small snippets of code to output
//data through UART or to provide small input adjustments
#define DEBUG 0

//Defines the usage of this chip. It may be one or multiple of the following roles:
//  Path Manager - Communicates with the GPS in order to provide a constant
//                  heading for the aircraft to follow.
//  Attitude Manager - Communicates with a IMU (kinematics sensor) in order to
//                     the desires Pitch, Roll, Yaw on the aircraft.
#define PATH_MANAGER 0
#define ATTITUDE_MANAGER !PATH_MANAGER

//Define this for competition, includes higher restrictions for safety
#define COMP_MODE 1

/* CHANGE THIS HEADER FILE WHEN MODIFYING THIS FOR A NEW PLANE OR VEHICLE */
#define ANACONDA_VEHICLE 1
#define QUAD_VEHICLE 0

#if ANACONDA_VEHICLE
#include "Anaconda.h"
#define FIXED_WING 1
#define COPTER 0
#elif QUAD_VEHICLE
#include "DisplayQuad.h"
#define COPTER 1
#define FIXED_WING 0
#endif

#if DEBUG
    #include "../Common/debug.h"
#endif