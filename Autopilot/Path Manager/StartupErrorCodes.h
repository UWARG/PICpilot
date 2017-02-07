/* 
 * File:   StartupErrorCodes.h
 * Author: Chris Hajduk
 *
 * Created on October 15, 2013, 8:33 PM
 */
#ifndef STARTUPERRORCODES_H
#define STARTUPERRORCODES_H

#define STARTUP_ERROR_BROWN_OUT_RESET (1 << 1)
#define STARTUP_ERROR_POWER_ON_RESET 1

//TODO: Add descriptions to these functions
void checkErrorCodes();
unsigned int getErrorCodes();

#endif