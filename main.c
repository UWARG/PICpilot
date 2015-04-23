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

#if PATH_MANAGER
#include "PathManager.h"
#endif
#if ATTITUDE_MANAGER
#include "AttitudeManager.h"
#endif
#if COMMUNICATION_MANAGER
#include "net.h"
#endif

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

//Debug Mode initialize communication with the serial port (Computer)
#if DEBUG
    initDebug();
#endif

#if PATH_MANAGER
    pathManagerInit();
#endif

#if ATTITUDE_MANAGER
    attitudeInit();
#endif

#if COMMUNICATION_MANAGER
    initDataLink();
#endif


checkErrorCodes();



    while (1) {

#if PATH_MANAGER
        pathManagerRuntime();
#endif

#if ATTITUDE_MANAGER
        attitudeManagerRuntime();
#endif

#if COMMUNICATION_MANAGER
        outboundBufferMaintenance();
        inboundBufferMaintenance();
#endif
        asm("CLRWDT");
    }
    return (EXIT_SUCCESS);
}

