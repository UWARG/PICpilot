/**
 * @file Timer.h
 * @author Serj Babayan
 * @created April 23, 2017, 2:09AM
 * 
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef TIMER_H
#define TIMER_H

/** This is the MIPS count of the chip */
#define CLOCK_FREQUENCY 16000000

/**
 * Initializes timer 1. Used as the system timer
 */
void initTimer1(void);

/**
 * @return Current system time in ms
 */
uint32_t getTime(void);

/**
 * Delay given a certain number of ms
 * @param ms
 */
void delay(uint32_t ms);

#endif