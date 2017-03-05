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
//#include "timer.h"

//Turns debug mode on or off. Usually contains small snippets of code to output
//data through UART or to provide small input adjustments
#define DEBUG 1

//  Path Manager - Communicates with the GPS in order to provide a constant
//                  heading for the aircraft to follow.

#define GPS_OLD 1 //1 Being the Old GPS (Uses SPI), and 0 Being the New GPS (Uses UART)

#if DEBUG
    #include "../Common/debug.h"
#endif