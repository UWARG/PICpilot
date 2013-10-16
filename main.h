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

//Define variables for global use in the code
#define TRUE	1
#define FALSE	0

#define DEBUG 0
#define STABILIZATION 1


#if DEBUG
    #include "UART1.h"
#endif