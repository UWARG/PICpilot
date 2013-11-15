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

#include <p33FJ256GP710.h>

//Define constants for global use in the code
#define TRUE	1
#define FALSE	0

#define DEBUG 1

#define STABILIZATION 1
#define ORIENTATION 1

//Defined Orientation Angle constants
#define YAW     0
#define PITCH   1
#define ROLL    2

//Defined in the opposite order for rates of rotation
//Be careful not to confuse the constants
#define YAW_RATE     2
#define PITCH_RATE   1
#define ROLL_RATE    0

#if DEBUG
    #include "UART1.h"
#endif