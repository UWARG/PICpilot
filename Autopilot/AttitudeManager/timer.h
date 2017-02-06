/* 
 * @file timer.h
 * @author Chris Hajduk
 * @created September 2, 2015, 3:37 PM
 * @description Provides functions for initializing the timers (only Timers 2 and 4 used)
 * as well as getting the current system time
 */

#ifndef TIMER_H
#define	TIMER_H

/**
 * Number of Timer2 ticks in a millisecond. To calculate this, take:
 * 1/(frequencyCPU/Timer2PreScaler)*TICKS_TO_MSEC should equal close to 0.001 or 1 ms
 * In this case, (1/(41Mhz/64))*642 == ~1ms
 * 
 * This value is used to accurately define the timer2 period (used for output pwm
 * frequency
 */
#define T2_TICKS_TO_MSEC 642

/**
 * Timer2 Period in ms. When the period is reached, the timer is reset. 
 * Since Timer2 is used in the output compare (PWM output), this period defines 
 * the period of the PWM as well. If you want 50Hz, set this as 20ms
 */
#define T2_PERIOD 20

/**
 * Initializes Timer2. Its used as a 16-bit timer. Used for PWM input and output management
 */
void initTimer2(void);

/**
 * Initializes Timer4. Used as a 16-bit, interrupt enabled, 1ms timer
 */
void initTimer4(void);

/**
 * Get current time in ms
 * @return ms
 */
long unsigned int getTime();

#endif