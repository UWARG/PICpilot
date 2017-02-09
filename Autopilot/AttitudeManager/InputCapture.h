/*
 * @file InputCapture.h
 * @author Chris Hajduk
 * @created March 4, 2013, 10:31 PM
 * @description This file provides the methods necessary to access the input capture
 * capabilities of the chip. In essence, it lets you get the raw, uncalibrated PWM values 
 * from the 8 available input compare channels
 * 
 * Channel 7 is specifically also configured as the UHF connection switch. An edge
 * detected on channel 7 will signify that the UHF is still alive by saving a timestamp
 * which can be compared later. This can be reconfigured to a different channel, however
 * the position of the timestamp save must be placed in a different interrupt service
 * routine.
 */

#ifndef INPUTCAPTURE_H
#define	INPUTCAPTURE_H

/**
 * Number of ms after the last detected edge on a channel before it can be assumed to be
 * disconnected
 */
#define PWM_ALIVE_THRESHOLD 100

/**
 * Initializes capture configuration of the PWM input channels. Make sure to initialize Timer2
 * before calling this!
 * @param initIC An 8-bit bit mask specifying which channels to enable (will enable interrupts on these)
 */
void initIC(unsigned char initIC);

/**
 * Gets the input capture value (in ticks) of all the channels
 * @param sys_time The system time in milliseconds. Used for detecting disconnected channels. A channel
 * that is disconnected will have a value of 0.
 * @return Array containing all the channel values
 */
unsigned int* getICValues(unsigned long int sys_time);

#endif
