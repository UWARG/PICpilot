/*
 * @file InputCapture.c
 * @created February 9, 2010, 10:53 AM
 */

#include <p33Fxxxx.h>
#include "InputCapture.h"
#include "timer.h"

/**
 * Number of timer ticks that indicate a sync pulse
 */
#define PPM_SYNC_TICKS (int)((float)(PPM_MIN_SYNC_TIME/1000)*T2_TICKS_TO_MSEC)

/**
 * Holds the capture start and end time so that we can compare them later. We can
 * only do 8 with PWM enabled. Otherwise
 */
#if USE_PPM
static unsigned int start_time[PPM_CHANNELS];
static unsigned int end_time[PPM_CHANNELS];
#else
static unsigned int start_time[8];
static unsigned int end_time[8];
#endif

/**
 * Interrupt flag for if new data is available and ready to read. This variable
 * is not used if using PPM
 */
#if !USE_PPM
static char new_data_available[8];
#endif

/**
 * The actual time between interrupts (in timer2 ticks, not ms)
 */
#if USE_PPM
static unsigned int capture_value[PPM_CHANNELS];
#else
static unsigned int capture_value[8];
#endif

/**
 * Last capture time in ms for all the channels. Used for detecting a channel/pwm
 * disconnect. If using PPM, we only store it as one variable, since we've only got
 * one physical connection
 */
#if USE_PPM
static unsigned long int ppm_last_capture_time;
#else
static unsigned long int last_capture_time[8];
#endif

/**
 * Used to keep track of the pulse position when PPM is enabled
 */
#if USE_PPM
static unsigned char ppm_index;
#endif

unsigned int* getICValues(unsigned long int sys_time)
{
    
#if USE_PPM
    /**
     * The actual calculation of comparison values is already done in the ISR
     * as part of the sync pulse detection, so we just return the values, unless
     * we detected a disconnect
     */
    if ((sys_time - ppm_last_capture_time) > PWM_ALIVE_THRESHOLD){
        int i = 0;
        for (i = 0; i < PPM_CHANNELS; i++){
            capture_value[i] = 0;
        }
    }
    return capture_value;
#else
    int channel;
    for (channel = 0; channel < 8; channel++) {
        /*
         * Check if we received any data from the channel within the threshold.
         * If not, set the channel input as 0 to indicate that its disconnected
         */
        if ((sys_time - last_capture_time[channel]) > PWM_ALIVE_THRESHOLD) {
            capture_value[channel] = 0;
        } else { //otherwise the channel is connected. Perform the same necessary calculations
            if (new_data_available[channel] == 1) {
                //If the second time is greater than the first time then we have not met roll over
                if (end_time[channel] > start_time[channel]) {
                    capture_value[channel] = end_time[channel] - start_time[channel];
                } else {
                    /*
                     * We've reached roll over. Add the maximum time (PR2) to the original start time,
                     * and add on the end time to find the total time. Note that the PR2 register stores
                     * the time period for Timer2, and is set at 20ms. After 20ms, the timer will
                     * reset. Since an average PWM width is 2-3ms max, this is more than sufficient.
                     */
                    capture_value[channel] = ((PR2 - start_time[channel]) + end_time[channel]);
                }
                new_data_available[channel] = 0;
            }
        }
    }
    return capture_value;
#endif
}

/**
 * Initializes interrupts for the specified channels. Sets Timer2
 * as the time base, and configures it so that interrupts occur on every rising and
 * falling edge
 * @param initIC An 8-bit bit mask specifying which channels to enable interrupts on
 */
void initIC(unsigned char initIC)
{
    //If using PPM, we want to unconditionally turn on channel 7
    #if USE_PPM
      initIC = 0b01000000;
      ppm_index = 0;
    #endif

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

#if USE_PPM
/**
* PPM Interrupt Service routine for Channel 1 for when PPM is enabled. Will trigger
* on any edge change on channel 1. Calculates the time between the last rise time
* and last fall time to determine if a PPM sync occured, used to keep track
* of the positions of the channels
*/
void __attribute__((__interrupt__, no_auto_psv)) _IC7Interrupt(void)
{
    static unsigned int time_diff;
    
    if (PORTDbits.RD14 == !PPM_INVERTED) { //If PPM inverted, check for fall (0). Otherwise check for rise (1)
        start_time[ppm_index] = IC7BUF;
    } else { //if PPM inverted, then if rise (1). Otherwise if fall
        end_time[ppm_index] = IC7BUF;
        
        //take into account for timer overflow
        if (end_time[ppm_index] > start_time[ppm_index]){
            time_diff = end_time[ppm_index] - start_time[ppm_index];   
        } else{
            time_diff = (PR2 - start_time[ppm_index]) + end_time[ppm_index];
        }
        
        if (time_diff >= PPM_SYNC_TICKS){ //if we just captured a sync pulse
            ppm_index = 0; 
        } else {
            capture_value[ppm_index] = time_diff;
            ppm_index = (ppm_index + 1) % PPM_CHANNELS; //make sure we don't overflow
        }
        ppm_last_capture_time = getTime(); //for detecting disconnect
    }

    /**
     * Clear the input compare buffer to avoid any issues when hot swapping PWM cables.
     * Without this, when hot-swapping PWM connections, you may get weird values (in the 10000's range)
     * when reading off of the connection. Note that in normal circumstances, the maximum size
     * of the buffer at any time will be 1, so this while loop should never execute. Its only when
     * you disconnect it and reconnect it that stuff gets weird.
     */
    while (IC1CONbits.ICBNE) { //while the ic buffer not empty flag is set
        IC1BUF; //read a value from the 4-size FIFO buffer
    }
    IFS0bits.IC1IF = 0; //reset the interrupt flag
}

#else
/**
 * PWM Interrupt Service Routines for when PPM is disabled. These will trigger
 * on an edge change on an enabled PWM channel.These functions will mark the new_data_available
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
        last_capture_time[0] = getTime();
    }

    /**
     * Clear the input compare buffer to avoid any issues when hot swapping PWM cables.
     * Without this, when hot-swapping PWM connections, you may get weird values (in the 10000's range)
     * when reading off of the connection. Note that in normal circumstances, the maximum size
     * of the buffer at any time will be 1, so this while loop should never execute. Its only when
     * you disconnect it and reconnect it that stuff gets weird.
     */
    while (IC1CONbits.ICBNE) { //while the ic buffer not empty flag is set
        IC1BUF; //read a value from the 4-size FIFO buffer
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
        last_capture_time[1] = getTime();
    }

    while (IC2CONbits.ICBNE) {
        IC2BUF;
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
        last_capture_time[2] = getTime();
    }

    while (IC3CONbits.ICBNE) {
        IC3BUF;
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
        last_capture_time[3] = getTime();
    }

    while (IC4CONbits.ICBNE) {
        IC4BUF;
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
        last_capture_time[4] = getTime();
    }

    while (IC5CONbits.ICBNE) {
        IC5BUF;
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
        last_capture_time[5] = getTime();
    }

    while (IC6CONbits.ICBNE) {
        IC6BUF;
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
        last_capture_time[6] = getTime();
    }

    while (IC7CONbits.ICBNE) {
        IC7BUF;
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
        last_capture_time[7] = getTime();
    }

    while (IC8CONbits.ICBNE) {
        IC8BUF;
    }
    IFS1bits.IC8IF = 0;
}
#endif
