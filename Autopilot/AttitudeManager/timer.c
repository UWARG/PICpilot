/*
 * File:   timer.c
 * Author: Chris Hajduk
 *
 * Created on July 19, 2015, 8:07 PM
 */

#include "main.h"
#include "timer.h"

long int time = 0;

//NEED A BETTER TIMER - THIS IS TOO SLOW. TIMER3?? Something accurate to 1ms.

//void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void){
//    //TODO: Disable this interrupt
//    //Timer Interrupt used for the control loops and data link which is accurate to 20ms
////    time += 20;
//    IFS0bits.T2IF = 0;
//}

void initTimer4(){
    T4CONbits.TON = 0; // Disable Timer
    T4CONbits.TCS = 0; // Select internal instruction cycle clock
    T4CONbits.TGATE = 0; // Disable Gated Timer mode
    T4CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
    TMR4 = 0x00; // Clear timer register
    PR4 = 642; // Load the period value
    IPC6bits.T4IP = 0x01; // Set Timer1 Interrupt Priority Level
    IFS1bits.T4IF = 0; // Clear Timer1 Interrupt Flag
    IEC1bits.T4IE = 1; // Enable Timer1 interrupt
    T4CONbits.TON = 1; // Start Timer
}

void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void){
    time += 1;
    IFS1bits.T4IF = 0;
}

long int getTime(){
    return time;
}