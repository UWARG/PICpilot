/* 
 * @file OutputCompare.c
 * @created February 9, 2010, 10:53 AM
 */

#include <p33Fxxxx.h>
#include "OutputCompare.h"
#include "InputCapture.h"

void setOCValue(unsigned int channel, unsigned int duty)
{
    switch(channel){
    case 0:
        OC1RS = duty;
        break;
    case 1:
        OC2RS = duty;
        break;
    case 2:
         OC3RS = duty;
         break;
    case 3:
        OC4RS = duty;
        break;
    case 4:
        OC5RS = duty;
        break;
    case 5:
        OC6RS = duty;
        break;
    case 6:
        OC7RS = duty;
        break;
    case 7:
        OC8RS = duty;
        break;
    }
}

void initOC(char OC)
{
    //Initialize each of the 8 OCs
    if (OC & 0b1) {
        OC1CONbits.OCM = 0b000; // Disable Output Compare Module )required to set it as something else)
        OC1RS = 0x00; // Write a duty cycle of 0 to start off with
        OC1CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
        OC1CONbits.OCM = 0b110; // Select the Output Compare mode (PWM without fault protection)
    }
    if (OC & 0b10) {
        OC2CONbits.OCM = 0b000;
        OC2RS = 0x00;
        OC2CONbits.OCTSEL = 0;
        OC2CONbits.OCM = 0b110;

    }
    if (OC & 0b100) {
        OC3CONbits.OCM = 0b000;
        OC3RS = 0x00;
        OC3CONbits.OCTSEL = 0;
        OC3CONbits.OCM = 0b110;
    }
    if (OC & 0b1000) {
        OC4CONbits.OCM = 0b000;
        OC4RS = 0x00;
        OC4CONbits.OCTSEL = 0;
        OC4CONbits.OCM = 0b110;
    }
    if (OC & 0b10000) {
        OC5CONbits.OCM = 0b000;
        OC5RS = 0x00;
        OC5CONbits.OCTSEL = 0;
        OC5CONbits.OCM = 0b110;
    }
    if (OC & 0b100000) {
        OC6CONbits.OCM = 0b000;
        OC6RS = 0x00;
        OC6CONbits.OCTSEL = 0;
        OC6CONbits.OCM = 0b110;
    }
    if (OC & 0b1000000) {
        OC7CONbits.OCM = 0b000;
        OC7RS = 0x00;
        OC7CONbits.OCTSEL = 0;
        OC7CONbits.OCM = 0b110;
    }
    if (OC & 0b10000000) {
        OC8CONbits.OCM = 0b000;
        OC8RS = 0x00;
        OC8CONbits.OCTSEL = 0;
        OC8CONbits.OCM = 0b110;
    }
}