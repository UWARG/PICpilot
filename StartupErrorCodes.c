/*
 * File:   StartupErrorCodes.c
 * Author: Chris Hajduk
 *
 * Created on October 15, 2013, 7:57 PM
 */
#include "main.h"

int lastRuntimeErrors = 0;

void checkErrorCodes(){
    lastRuntimeErrors = 0;
    if (RCONbits.TRAPR == 1) {
        if (DEBUG){
            UART1_SendString("TRAP Reset Occurred");
        }
        lastRuntimeErrors += (1 << 10);
        RCONbits.TRAPR = 0;
    }

    if (RCONbits.IOPUWR == 1) {
        if (DEBUG){
            UART1_SendString("Illegal Opcode Reset Occurred");
        }
        lastRuntimeErrors += (1 << 9);
        RCONbits.IOPUWR = 0;
    }

    if (RCONbits.VREGS == 1) {
        if (DEBUG){
            UART1_SendString("Voltage Reg Reset Occurred");
        }
        lastRuntimeErrors += (1 << 8);
        RCONbits.VREGS = 0;
    }

    if (RCONbits.EXTR == 1) {
        if (DEBUG){
            UART1_SendString("External Reset Occurred");
        }
        lastRuntimeErrors += (1 << 7);
        RCONbits.EXTR = 0;
    }

    if (RCONbits.SWR == 1) {
        if (DEBUG){
            UART1_SendString("Software Reset Occurred");
        }
        lastRuntimeErrors += (1 << 6);
        RCONbits.SWR = 0;
    }

    if (RCONbits.WDTO == 1) {
        if (DEBUG){
        UART1_SendString("Software WDT Reset Occurred");
        }
        lastRuntimeErrors += (1 << 5);
        RCONbits.WDTO = 0;
    }

    if (RCONbits.SLEEP == 1) {
        if (DEBUG){
            UART1_SendString("Sleep Mode Reset Occurred");
        }
        lastRuntimeErrors += (1 << 4);
        RCONbits.SLEEP = 0;
    }

    if (RCONbits.IDLE == 1) {
        if (DEBUG){
            UART1_SendString("Idle Mode Reset Occurred");
        }
        lastRuntimeErrors += (1 << 3);
        RCONbits.IDLE = 0;
    }

    if (RCONbits.BOR == 1) {
        if (DEBUG){
            UART1_SendString("Brown Out Reset Occurred");
        }
        lastRuntimeErrors += (1 << 2);
        RCONbits.BOR = 0;
    }

    if (RCONbits.POR == 1) {
        if (DEBUG){
            UART1_SendString("Power On Reset Occurred");
        }
        lastRuntimeErrors += 1;
        RCONbits.POR = 0;
    }
}
int getErrorCodes(){
   return lastRuntimeErrors;
}
