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

#include "AttitudeManager.h"
#include "net.h"
#include "../Common/clock.h"
#include "../Common/Utilities/Logger.h"

/*
 * 
 */
//Perform random clock things that came with the Sample Code (Keep in code)
_FOSCSEL(FNOSC_FRC); // Internal FRC oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);
_FWDT(FWDTEN_ON & WDTPOST_PS4096 & WDTPRE_PR128);
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
    initLogger();
#endif
    
    checkErrorCodes();
    attitudeInit();
    
    while (1) {
        //Continue running the state machine forever.
        StateMachine(STATEMACHINE_MAINLOOP);
    }
    return (EXIT_SUCCESS);
}

