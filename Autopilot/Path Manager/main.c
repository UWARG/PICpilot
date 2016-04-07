/*
 * File:   main.c
 * Author: Chris Hajduk
 *
 * Created on January 26, 2014, 4:39 PM
 */

/*************************************************************************
 * PicPilot Software
 *
 * Originally developed by Chris Hajduk, Mitchell Hatfield, Andrew Crichton, et al. in 2013-2014.
 * Rev. Spike+ V1.0
 *
 * See documentation for further details.
 **************************************************************************/

#include "main.h"
#include "StartupErrorCodes.h"
#include "PathManager.h"
#include "../common/clock.h"

/*
 * 
 */
//Perform random clock things that came with the Sample Code (Keep in code)
_FOSCSEL(FNOSC_FRC); // Internal FRC oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);
_FWDT(FWDTEN_ON & WDTPOST_PS2048 & WDTPRE_PR128); //32,128
// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystanl

/*
 *
 */
int main(int argc, char** argv) {

    useFRCPLLClock();

    // Init intercom pins as digital pins
    AD1PCFGHbits.PCFG20 = 1;
    AD1PCFGHbits.PCFG21 = 1;
    AD1PCFGLbits.PCFG4 = 1;
    AD1PCFGLbits.PCFG5 = 1;
    AD2PCFGLbits.PCFG4 = 1;
    AD2PCFGLbits.PCFG5 = 1;
    
//Debug Mode initialize communication with the serial port (Computer)
#if DEBUG
    initDebug();
#endif

pathManagerInit();
checkErrorCodes();



    while (1) {
        pathManagerRuntime();
        asm("CLRWDT");
    }
    return (EXIT_SUCCESS);
}

