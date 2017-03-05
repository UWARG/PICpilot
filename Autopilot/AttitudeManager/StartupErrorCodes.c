/*
 * File:   StartupErrorCodes.c
 * Author: Chris Hajduk
 *
 * Created on October 15, 2013, 7:57 PM
 */
#include "main.h"
#include "StartupErrorCodes.h"

unsigned int lastStartupErrors = 0;

void checkErrorCodes(){
    lastStartupErrors = 0;
    if (RCONbits.TRAPR == 1) {
        error("TRAP Reset Occurred");
        lastStartupErrors += (1 << 9);
        RCONbits.TRAPR = 0;
    }

    if (RCONbits.IOPUWR == 1) {
        error("Illegal Opcode Reset Occurred");
        lastStartupErrors += (1 << 8);
        RCONbits.IOPUWR = 0;
    }

    if (RCONbits.VREGS == 1) {
        error("Voltage Reg Reset Occurred");
        lastStartupErrors += (1 << 7);
        RCONbits.VREGS = 0;
    }

    if (RCONbits.EXTR == 1) {
        error("External Reset Occurred");
        lastStartupErrors += (1 << 6);
        RCONbits.EXTR = 0;
    }

    if (RCONbits.SWR == 1) {
        error("Software Reset Occurred");
        lastStartupErrors += (1 << 5);
        RCONbits.SWR = 0;
    }

    if (RCONbits.WDTO == 1) {
        error("Software WDT Reset Occurred");
        lastStartupErrors += (1 << 4);
        RCONbits.WDTO = 0;
    }

    if (RCONbits.SLEEP == 1) {
        error("Sleep Mode Reset Occurred");
        lastStartupErrors += (1 << 3);
        RCONbits.SLEEP = 0;
    }

    if (RCONbits.IDLE == 1) {
        error("Idle Mode Reset Occurred");
        lastStartupErrors += (1 << 2);
        RCONbits.IDLE = 0;
    }

    if (RCONbits.BOR == 1) {
        error("Brown Out Reset Occurred");
        lastStartupErrors += (1 << 1);
        RCONbits.BOR = 0;
    }

    if (RCONbits.POR == 1) {
        error("Power On Reset Occurred");
        lastStartupErrors += 1;
        RCONbits.POR = 0;
    }
}
unsigned int getStartupErrorCodes(){
   return lastStartupErrors;
}
