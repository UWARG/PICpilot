/* 
 * @file OutputCompare.h
 * @author Chris Hajduk
 * @created March 3, 2013, 12:42 AM
 * @desription Provides functions for utilizing the output compare feature of the
 * chip. Basically lets you set the PWM duty cycle for one of the 8 output channels
 */

#ifndef OUTPUTCAPTURE_H
#define	OUTPUTCAPTURE_H

#include <stdint.h>

/**
 * Initializes the output compare registers and pins for PWM output. Writes a 1.5 ms
 * duty cycle on the selected channels to start off with
 * @param OC 8-bit bit mask indicating which pins to enable for output
 */
void initOC(char OC);

/**
 * Sets the PWM of a given output pin
 * @param channel Number from 0-7
 * @param time Time/duty cycle in Timer2 ticks, not ms
 */
void setOCValue(unsigned int channel, unsigned int duty);


/**
 * Retrieve the set values for the output compare
 * @return ms output of all the channels in Timer2 ticks. Returned array will be of size 8
 */
uint16_t* getOCValues(void);

#endif