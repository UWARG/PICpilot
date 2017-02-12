/*
 * @file InputCapture.h
 * @author Chris Hajduk
 * @created March 4, 2013, 10:31 PM
 * @description This file provides the methods necessary to access the input capture
 * capabilities of the chip. In essence, it lets you get the raw, uncalibrated PWM values
 * from the 8 available input compare channels
 *
 * If using PPM, by default the input channel on the Picpilot will be channel 7.
 */

#ifndef INPUTCAPTURE_H
#define	INPUTCAPTURE_H

/**
* Use this setting to disable or enable PPM. PPM is currently only configured
* for channel 1. If disabled, regular PWM via the 8 channel inputs is used
*/
#define USE_PPM 1

/**
 * If the PPM signal is inverted. Some receivers, such as the ezUHF have an inverted
 * PPM signal, so set this to 1. If using the OrangeRX, set this to 0
 */
#define PPM_INVERTED 0

/**
* How many channels are expected to be in a single PPM frame
*/
#define PPM_CHANNELS 8

/**
 * If using PPM, this is the sync time between frames in us. Required so that
 * the pic pilot can tell frames apart. A value of 3000us is recommended. Note
 * that this is the minimum sync time. It may/will be larger, especially if using
 * less than 12 PPM channels
 */
#define PPM_MIN_SYNC_TIME 3000

/**
 * Number of ms after the last detected edge on a channel before it can be assumed to be
 * disconnected
 */
#define PWM_ALIVE_THRESHOLD 100

/**
 * Initializes capture configuration of the PWM input channels. Make sure to initialize Timer2
 * before calling this! Disabled channels will not have interrupts called on them, and
 * disconnect detection will also be disabled
 * @param channel number from 0-7
 * @return Value of the IC channel. This is in Timer2 ticks, not ms!
 * The timer module defines number of ticks in a ms
 */
void initIC(unsigned char initIC);

/**
 * Gets the input capture value (in Timer2 ticks) of all the channels
 * @param sys_time The system time in milliseconds. Used for detecting disconnected channels. A channel
 * that is disconnected will have a value of 0.
 * @return Array containing all the channel values
 */
unsigned int* getICValues(unsigned long int sys_time);

#endif
