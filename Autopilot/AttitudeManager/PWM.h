/**
 * @file PWM.h
 * @author Chris Hajduk
 * @created August 21, 2014, 11:45 PM
 * @description This module abstracts the PWM input and output functionalities of the
 * chip. It combines the InputCompare and OutputCompare module, and provides functions
 * for scaling/calibrating PWM values, reading scaled PWM values, and setting scaled PWM values
 * for any of the 8 available PWM channels on the chip.
 * 
 * Note that this is the module that scales the PWM inputs from the RC controller
 * range to the -1024 to 1024 range, as well as scales the outputs in the opposite manner.
 */

#ifndef PWM_H
#define	PWM_H

/**
 * Number of channels available. This is not meant to be configurable, so don't change it
 */
#define NUM_CHANNELS 8

/**
 * The inputs received from the RC controlled will be scaled according to these 
 * MIN_PWM and MAX_PWM values. In addition, the pre-scaled outputs must be within
 * this range.
 */
#define MAX_PWM 1024
#define MIN_PWM -1024

/**
 * Maximum and minimum limits received from the controller. These should be calibrated,
 * and the scaling from these values to the MAX and MIN PWM values will depend on these.
 * Note that these will be used as the default/initial values, however the actual scaling values
 * can be set by the groundstation
 */
#define UPPER_PWM 1284
#define LOWER_PWM 642

#define SP_RANGE MAX_PWM//(UPPER_PWM - MIDDLE_PWM)

/**
 * Initializes the PWM input and output channels. Also initializes Timer2
 * @param inputChannels 8-bit bit mask indicating which inputs to initialize (probably want to send 0xFF for all)
 * @param outputChannels 8-bit bit mask indicating which outputs to initialize
 */
void initPWM(char inputChannels, char outputChannels);

/**
 * Gets the scaled PWM values in the rnage of MIN_PWM and MAX_PWM from all the channels.
 * Make sure that initPWM is called before calling this, otherwise the results will be useless
 * @return An integer array of size 8 containing the PWM values for all the channels. Note that
 * this array is zero-indexed, so channel 1 is index 0 (unlike the getPWM function)
 */
int* getPWMArray();

/*****************************************************************************
 * Function: void setPWM(unsigned int channel, int pwm);
 *
 * Preconditions: The PWM outputs must have been initialized for valid output.
 *
  Overview: Sets the output through the output compare pins on the device.
 *
 * Input: unsigned int channel -> The channel that is being set.
 *        int pwm -> The value (usually between +-1024) that is to be set to the
 *                  output compare pins.
 *
 * Output: None.
 *
 *****************************************************************************/
/**
 * Sets the PWM output of a particular output. Make sure initPWM is called before
 * this, otherwise unexpected behavior will occur
 * @param channel Number from 1-8 (not zero-indexed) indicating the channel. Values outside
 *      of this range will be ignored
 * @param pwm Value from MIN_PWM to MAX_PWM range (or -1024 to 1024). Values outside
 *      of this range will be ignored
 */
void setPWM(unsigned int channel, int pwm);

/**
 * Gets an array of all the set PWM values for all the 8 channel outputs
 * @return An array of size 8 containing the latest set PWM value in the range of MIN_PWM and MAX_PWM
 *      for all the channels. Note the array is 0-indexed, so channel 1 is array index 0
 */
int* getPWMOutputs();

/**
 * Calibrates the input range and trim of a PWM channel input
 * @param channel Number from 1-8 indicating the channel (not zero-based)
 * @param signalScaleFactor The scale factor that the input will be scaled by
 * @param signalOffset The signal offset for the signal, or trim
 * 
 * @note The signal factor and signal offset should be values such that:
 *      (any_received_pwm_value - signal_offset)*scale_factor ~= -1024 - 1024 (MIN_PWM - MAX_PWM)
 */
void calibratePWMInputs(unsigned int channel, float signalScaleFactor, unsigned int signalOffset);

/**
 * Calibrates the output range and trim of a PWM output
 * @param channel Number from 1-8 indicating the channel (not zero-based)
 * @param signalScaleFactor The scale factor that the input will be scaled by
 * @param signalOffset The signal offset for the signal, or trim
 * 
 * @note The signal factor and signal offset should be values such that:
 *      (any_PWM_value*scale_factor + signal_offset =~ Servo Range PWM
 *      Where any any_PWM_value value will be in the range of -1024 to 1024 (MIN_PWM and MAX_PWM)
 */
void calibratePWMOutputs(unsigned int channel, float signalScaleFactor, unsigned int signalOffset);

#endif