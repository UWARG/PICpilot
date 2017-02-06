/*
 * File:   timer.c
 * Author: Chris Hajduk
 *
 * Created on July 19, 2015, 8:07 PM
 */

#include "main.h"
#include "timer.h"

static unsigned long int time = 0;

/**
 * Initializes Timer2. Its used as a 16-bit timer
 */
void initTimer2(void)
{
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    TMR2 = 0x00; // Clear timer register
    T2CONbits.TCKPS = 0x02; //1:64 scaler
    PR2 = T2_PERIOD * T2_TICKS_TO_MSEC; //set the period
    IPC1bits.T2IP = 0x01; // Set Timer 2 Interrupt Priority Level - Lowest
    IFS0bits.T2IF = 0; // Clear Timer 2 Interrupt Flag
    IEC0bits.T2IE = 0; // Disable Timer 2 interrupt
    T2CONbits.TON = 1; // Start Timer
}

/**
 * Initializes Timer4 as a 1ms, 16-bit timer
 */
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

/**
 * Timer4 interrupt. Executed every ms
 */
void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void){
    time += 1;
    IFS1bits.T4IF = 0;
}

long unsigned int getTime(){
    return time;
}