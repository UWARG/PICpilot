/*
 * File:   StartupErrorCodes.c
 * Author: Chris Hajduk
 *
 * Created on October 15, 2013, 7:57 PM
 */
#include "main.h"
#include "StartupErrorCodes.h"

unsigned int lastRuntimeErrors = 0;

void checkErrorCodes(){
    lastRuntimeErrors = 0;
    if (RCONbits.TRAPR == 1) {
#if DEBUG
        error("TRAP Reset Occurred");
#endif
        lastRuntimeErrors += (1 << 9);
        RCONbits.TRAPR = 0;
    }

    if (RCONbits.IOPUWR == 1) {
#if DEBUG
            error("Illegal Opcode Reset Occurred");
#endif
        lastRuntimeErrors += (1 << 8);
        RCONbits.IOPUWR = 0;
    }

    if (RCONbits.VREGS == 1) {
#if DEBUG
            error("Voltage Reg Reset Occurred");
#endif
        lastRuntimeErrors += (1 << 7);
        RCONbits.VREGS = 0;
    }

    if (RCONbits.EXTR == 1) {
#if DEBUG
            error("External Reset Occurred");
#endif
        lastRuntimeErrors += (1 << 6);
        RCONbits.EXTR = 0;
    }

    if (RCONbits.SWR == 1) {
#if DEBUG
            error("Software Reset Occurred");
#endif
        lastRuntimeErrors += (1 << 5);
        RCONbits.SWR = 0;
    }

    if (RCONbits.WDTO == 1) {
#if DEBUG
            error("Software WDT Reset Occurred");
#endif
        lastRuntimeErrors += (1 << 4);
        RCONbits.WDTO = 0;
    }

    if (RCONbits.SLEEP == 1) {
#if DEBUG
            error("Sleep Mode Reset Occurred");
#endif
        lastRuntimeErrors += (1 << 3);
        RCONbits.SLEEP = 0;
    }

    if (RCONbits.IDLE == 1) {
#if DEBUG
            error("Idle Mode Reset Occurred");
#endif
        lastRuntimeErrors += (1 << 2);
        RCONbits.IDLE = 0;
    }

    if (RCONbits.BOR == 1) {
#if DEBUG
            error("Brown Out Reset Occurred");
#endif
        lastRuntimeErrors += (1 << 1);
        RCONbits.BOR = 0;
    }

    if (RCONbits.POR == 1) {
#if DEBUG
            error("Power On Reset Occurred");
#endif
        lastRuntimeErrors += 1;
        RCONbits.POR = 0;
    }
}
unsigned int getErrorCodes(){
   return lastRuntimeErrors;
}
