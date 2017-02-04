/*
 * @file InputCapture.c
 * @created February 9, 2010, 10:53 AM
 */

#include <p33Fxxxx.h>
#include "InputCapture.h"
#include "OutputCompare.h"
#include "timer.h"

/**
 * Raw captures of timer1 and timer2 for all 8 channels
 */
static unsigned int start_time[8];
static unsigned int end_time[8];

/**
 * Interrupt flag for if new data is available
 */
static char new_data_available[8];

/**
 * The actual time between interrupts (or pulses, in ms)
 */
static unsigned int capture_value[8];

static void initInputCapture(char initIC);
static void calculateICValue(unsigned char channel);

void initIC(char initIC)
{
    initInputCapture(initIC);
    initTimer2();
}
unsigned int* getICValues()
{
    //Calculate and Update the Input Values
    char channel;
    for (channel = 0; channel < 8; channel++) {
        calculateICValue(channel);
    }
    return capture_value;
}

unsigned int getICValue(unsigned char channel)
{
    calculateICValue(channel);
    return capture_value[channel];
}

static void calculateICValue(unsigned char channel)
{
    if (new_data_available[channel] == 1) {
        //If the second time is greater than the first time then we have not met roll over
        if (end_time[channel] > start_time[channel]) {
            capture_value[channel] = end_time[channel] - start_time[channel];
        } else {
            /*
             * We've reached roll over. Add the maximum time (PR2) to the original start time,
             * and add on the end time to find the total time. Note that the PR2 register stores
             * the time period for Timer2, and is set at 20ms (in initialization function). 
             * Since an average PWM width is 2-3ms max, this is more than sufficient.
             */
            capture_value[channel] = ((PR2 - start_time[channel]) + end_time[channel]);
        }

        new_data_available[channel] = 0;
    }
}

/**
 * Initializes interrupts for the specified channels. Sets Timer2
 * as the time base, and configures it so that interrupts occur on every rising and
 * falling edge
 * @param initIC An 8-bit bit mask specifying which channels to enable interrupts on
 */
static void initInputCapture(char initIC)
{
    if (initIC & 0b01) {
        IC1CONbits.ICM = 0b00; // Disable Input Capture 1 module (required to change it)
        IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
        
        /**
         * Generate capture event on every Rising and Falling edge
         * Note that the ICI register is ignored when ICM is in edge detection mode (001)
         */
        IC1CONbits.ICM = 0b001; 

        // Enable Capture Interrupt And Timer2
        IPC0bits.IC1IP = 7; // Setup IC1 interrupt priority level - Highest
        IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
        IEC0bits.IC1IE = 1; // Enable IC1 interrupt
    }
    if (initIC & 0b10) {
        IC2CONbits.ICM = 0b00;
        IC2CONbits.ICTMR = 1;
        IC2CONbits.ICM = 0b001;

        IPC1bits.IC2IP = 7;
        IFS0bits.IC2IF = 0;
        IEC0bits.IC2IE = 1;
    }
    if (initIC & 0b100) {
        IC3CONbits.ICM = 0b00;
        IC3CONbits.ICTMR = 1;
        IC3CONbits.ICM = 0b001;

        IPC9bits.IC3IP = 7;
        IFS2bits.IC3IF = 0;
        IEC2bits.IC3IE = 1;
    }
    if (initIC & 0b1000) {
        IC4CONbits.ICM = 0b00;
        IC4CONbits.ICTMR = 1;
        IC4CONbits.ICM = 0b001;

        IPC9bits.IC4IP = 7;
        IFS2bits.IC4IF = 0;
        IEC2bits.IC4IE = 1;
    }
    if (initIC & 0b10000) {
        IC5CONbits.ICM = 0b00;
        IC5CONbits.ICTMR = 1;
        IC5CONbits.ICM = 0b001;

        IPC9bits.IC5IP = 7;
        IFS2bits.IC5IF = 0;
        IEC2bits.IC5IE = 1;
    }
    if (initIC & 0b100000) {
        IC6CONbits.ICM = 0b00;
        IC6CONbits.ICTMR = 1;
        IC6CONbits.ICM = 0b001;

        IPC10bits.IC6IP = 7;
        IFS2bits.IC6IF = 0;
        IEC2bits.IC6IE = 1;
    }
    if (initIC & 0b1000000) {
        IC7CONbits.ICM = 0b00;
        IC7CONbits.ICTMR = 1;
        IC7CONbits.ICM = 0b001;

        IPC5bits.IC7IP = 7;
        IFS1bits.IC7IF = 0;
        IEC1bits.IC7IE = 1;
    }
    if (initIC & 0b10000000) {
        IC8CONbits.ICM = 0b00;
        IC8CONbits.ICTMR = 1;
        IC8CONbits.ICM = 0b001;

        IPC5bits.IC8IP = 7;
        IFS1bits.IC8IF = 0;
        IEC1bits.IC8IE = 1;
    }
}

/**
 * Below are the configured interrupt handler functions for when there is an edge
 * change on an enabled PWM channel. These functions will mark the new_data_available
 * bits, as well as set/save the appropriate timer values.
 * 
 * If a high value is detected on an interrupt, it means we went from 0->1, so we mark
 * the start time. Otherwise, we mark the end time. In the latter case, we'll also mark
 * the data available bit as either 1 or 0. If we went from 0->1, we'll mark the data as not
 * ready. If it went from 0->1, we'll mark it as ready.
 */
//Input Capture #1 Interrupt Function

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{
    if (PORTDbits.RD8 == 1) { // if IC signal is goes 0 --> 1
        start_time[0] = IC1BUF;
        new_data_available[0] = 0; //we should do this so that we don't parse partial values (with wrong start time)
    } else {
        end_time[0] = IC1BUF;
        new_data_available[0] = 1;
    }
    IFS0bits.IC1IF = 0; //reset the interrupt flag
}

//Input Capture #2 Interrupt Function

void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{
    if (PORTDbits.RD9 == 1) {
        start_time[1] = IC2BUF;
        new_data_available[1] = 0;
    } else {
        end_time[1] = IC2BUF;
        new_data_available[1] = 1;
    }
    IFS0bits.IC2IF = 0;
}

//Input Capture #3 Interrupt Function

void __attribute__((__interrupt__, no_auto_psv)) _IC3Interrupt(void)
{
    if (PORTDbits.RD10 == 1) {
        start_time[2] = IC3BUF;
        new_data_available[2] = 0;
    } else {
        end_time[2] = IC3BUF;
        new_data_available[2] = 1;
    }
    IFS2bits.IC3IF = 0;
}

//Input Capture #4 Interrupt Function

void __attribute__((__interrupt__, no_auto_psv)) _IC4Interrupt(void)
{
    if (PORTDbits.RD11 == 1) {
        start_time[3] = IC4BUF;
        new_data_available[3] = 0;
    } else {
        end_time[3] = IC4BUF;
        new_data_available[3] = 1;
    }
    IFS2bits.IC4IF = 0;
}

//Input Capture #5 Interrupt Function

void __attribute__((__interrupt__, no_auto_psv)) _IC5Interrupt(void)
{
    if (PORTDbits.RD12 == 1) {
        start_time[4] = IC5BUF;
        new_data_available[4] = 0;
    } else {
        end_time[4] = IC5BUF;
        new_data_available[4] = 1;
    }
    IFS2bits.IC5IF = 0;
}

//Input Capture #6 Interrupt Function

void __attribute__((__interrupt__, no_auto_psv)) _IC6Interrupt(void)
{
    if (PORTDbits.RD13 == 1) {
        start_time[5] = IC6BUF;
        new_data_available[5] = 0;
    } else {
        end_time[5] = IC6BUF;
        new_data_available[5] = 1;
    }
    IFS2bits.IC6IF = 0;
}

//Input Capture #7 Interrupt Function

void __attribute__((__interrupt__, no_auto_psv)) _IC7Interrupt(void)
{
    if (PORTDbits.RD14 == 1) {
        start_time[6] = IC7BUF;
        new_data_available[6] = 0;
    } else {
        end_time[6] = IC7BUF;
        new_data_available[6] = 1;
    }
    IFS1bits.IC7IF = 0;
}

//Input Capture #8 Interrupt Function

void __attribute__((__interrupt__, no_auto_psv)) _IC8Interrupt(void)
{
    if (PORTDbits.RD15 == 1) {
        start_time[7] = IC8BUF;
        new_data_available[7] = 0;
    } else {
        end_time[7] = IC8BUF;
        new_data_available[7] = 1;
    }
    IFS1bits.IC8IF = 0;
}
