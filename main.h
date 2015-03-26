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
#include <p33FJ256GP710.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//Turns debug mode on or off. Usually contains small snippets of code to output
//data through UART or to provide small input adjustments
#define DEBUG 0

//Defines the usage of this chip. It may be one or multiple of the following roles:
//  Path Manager - Communicates with the GPS in order to provide a constant
//                  heading for the aircraft to follow.
//  Attitude Manager - Communicates with a IMU (kinematics sensor) in order to
//                      the desires Pitch, Roll, Yaw on the aircraft.
//  Communication Manager - Provides network communication (uplink/downlink) between the aircraft and
//                          the ground station (or any other data link).
#define PATH_MANAGER 0
#define ATTITUDE_MANAGER !PATH_MANAGER
#define COMMUNICATION_MANAGER !PATH_MANAGER

//Define constants for global use in the code
#define TRUE	1
#define FALSE	0

//Mathematical Constants
#define PI 3.14159265

//Basic Mathematical Conversions
#define deg2rad(DEG) ((DEG) * PI/180.0)
#define rad2deg(RAD) ((RAD) * 180.0/PI)

#if DEBUG
    #include "UART1.h"
#endif