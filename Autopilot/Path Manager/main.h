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

//  Path Manager - Communicates with the GPS in order to provide a constant
//                  heading for the aircraft to follow.

#define GPS_OLD 1 //1 Being the Old GPS (Uses SPI), and 0 Being the New GPS (Uses UART)

//Define constants for global use in the code
#define TRUE	1
#define FALSE	0

//Mathematical Constants
#define PI 3.14159265

//Basic Mathematical Conversions
#define deg2rad(DEG) ((DEG) * PI/180.0)
#define rad2deg(RAD) ((RAD) * 180.0/PI)

#if DEBUG
    #include "debug.h"
#endif