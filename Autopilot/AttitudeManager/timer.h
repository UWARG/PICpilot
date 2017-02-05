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
 * Initializes Timer2. Its used as a 16-bit timer. Used for PWM input management
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
long int getTime();

#endif