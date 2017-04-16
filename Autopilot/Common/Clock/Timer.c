/*
 * File:   timer.c
 * Author: Chris Hajduk
 *
 * Created on July 19, 2015, 8:07 PM
 */

#include "./Timer.h"
#include <p33FJ256GP710A.h>

static volatile uint32_t time_ms = 0;

/**
 * Initializes Timer2. Its used as a 16-bit timer
 */
void initTimer2(void)
{
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0x02; //1:64 scaler
    TMR2 = 0x00; // Clear timer register
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
    T4CONbits.TCKPS = 0b01; // Select 1:8 Prescaler
    T4CONbits.T32 = 0; //use as single 64-bit timer (uses Timer5)
    TMR4 = 0x00; // Clear timer register
    PR4 = T4_TICKS_TO_MSEC; // Load the period value
    IPC6bits.T4IP = 0x01; // Set Timer 4 Interrupt Priority Level
    IFS1bits.T4IF = 0; // Clear Timer 4 Interrupt Flag
    IEC1bits.T4IE = 1; // Enable Timer 4 interrupt
    T4CONbits.TON = 1; // Start Timer
}

/**
 * Timer4 interrupt. Executed every ms
 */
void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void){
    time_ms += 1;
    IFS1bits.T4IF = 0;
}

uint32_t getTime(){
    return time_ms;
}

uint64_t getTimeUs(){
    return ((uint64_t)time_ms)*1000 + TMR4;
}
