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

#include "main.h"

/**
 * Initializes capture configuration of the PWM input channels.
 * @param initIC An 8-bit bitmask specifying which channels to enable (will enable interrupts on these)
 */
void initIC(char initIC);

/**
 * Gets the input capture value (in ticks) of all the channels
 * @return Array containing all the channel values
 */
unsigned int* getICValues();

/**
 * Gets the input capture value of a specific value
 * @param channel number from 0-7
 * @return Value of the IC channel. This is in Timer2 ticks, not ms! 
 * The timer module defines number of ticks in a ms
 */
unsigned int getICValue(unsigned char channel);

/**
 * Get the last system time an edge was detected on channel 7. Should be used
 * for detecting a UHF disconnect
 * @return System time in ms since the last detected edge/data on channel 7
 */
unsigned long int getICLastCapturedTime(void);

#endif
