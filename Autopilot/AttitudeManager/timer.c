/*
 * File:   timer.c
 * Author: Chris Hajduk
 *
 * Created on July 19, 2015, 8:07 PM
 */

#include "main.h"
#include "timer.h"

long int time = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void){
    //Timer Interrupt used for the control loops and data link which is accurate to 20ms
    time += 20;
    IFS0bits.T2IF = 0;
}

long int getTime(){
    return time;
}