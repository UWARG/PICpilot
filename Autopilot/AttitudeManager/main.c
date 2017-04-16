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
#include "../Common/Clock/Clock.h"
#include "../Common/Utilities/Logger.h"
#include "delay.h"

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
    
    //as we plug in the picpilot, there may be intermittent power from the initial contact of the
    //power plug which the sensor drivers don't like. This delay is meant to stop communication of sensors 
    //until we know we're getting constant power
    Delay(100);
    
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

