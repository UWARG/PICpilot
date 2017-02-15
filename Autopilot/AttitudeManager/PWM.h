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
 * 
 * Also note that channel 7 input by default is set as the UHF keep alive channel,
 * and is tracked to for detecting UHF loss
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
 * Used in some of the calculations
 */
#define HALF_PWM_RANGE (MAX_PWM - MIN_PWM)/2

/**
 * Maximum and minimum limits received from the controller. These should be calibrated,
 * and the scaling from these values to the MAX and MIN PWM values will depend on these.
 * Note that these will be used as the default/initial values, however the actual scaling values
 * can be set by the ground station
 */
#define UPPER_PWM 1284
#define LOWER_PWM 642

/**
 * The middle of the PWM range of the RC controller. This is used as the initial
 * offset in the output and input scaling calculations
 */
#define MIDDLE_PWM (int)(((UPPER_PWM - LOWER_PWM)/2) + LOWER_PWM)

/**
 * Initial scale factors used for scaling the RC inputs to the MIN_PWM - MAX_PWM range,
 * and scaling outputs in the same range to the RC input range suitable for servos.
 * These scale factors should be reconfigured by the ground station, hence they are just the DEFAULT
 */
#define DEFAULT_INPUT_SCALE_FACTOR (MAX_PWM/(float)(UPPER_PWM - MIDDLE_PWM))
#define DEFAULT_OUTPUT_SCALE_FACTOR ((float)(UPPER_PWM - MIDDLE_PWM)/MAX_PWM) //its really just 1/INPUT_DEFAULT_SCALE_FACTOR, but this makes it clearer why

/**
 * Value to give to a channel thats disconnected. Used to easily let the
 * ground station operator know that the input is disconnected, and so that you don't
 * get random PWM input values if you have specific scaling
 * factors and offsets set for a particular channel
 */
#define DISCONNECTED_PWM_VALUE -10000

/**
 * Shortcuts for the applicable PWM statuses
 */
#define PWM_STATUS_UHF_LOST 0xFF
#define PWM_STATUS_OK 0

/**
 * Initializes the PWM input and output channels. Also initializes Timer2
 * @param inputChannels 8-bit bit mask indicating which inputs to initialize (probably want to send 0xFF for all)
 * @param outputChannels 8-bit bit mask indicating which outputs to initialize
 */
void initPWM(unsigned char inputChannels,unsigned char outputChannels);

/**
 * Gets the scaled PWM values in the range of MIN_PWM and MAX_PWM from all the channels.
 * Make sure that initPWM is called before calling this, otherwise the results will be useless
 * @param sys_time System time in ms used for detecting disconnects
 * @return An integer array of size 8 containing the PWM values for all the channels. Note that
 * this array is zero-indexed, so channel 1 is index 0 (unlike the getPWM function)
 */
int* getPWMArray(unsigned long int sys_time);

/**
 * Sets the PWM output of a particular output. Make sure initPWM is called before
 * this, otherwise unexpected behavior will occur
 * @param channel Number from 1-8 (not zero-indexed) indicating the channel. Values outside
 *      of this range will be ignored
 * @param pwm Value from MIN_PWM to MAX_PWM range (or -1024 to 1024). Values outside
 *      of this range will be ignored
 */
void setPWM(unsigned int channel, int pwm);

// TODO: doc
void setAllPWM(int* pwms);

/**
 * Gets an array of all the set PWM values for all the 8 channel outputs
 * @return An array of size 8 containing the latest set PWM value in the range of MIN_PWM and MAX_PWM
 *      for all the channels. Note the array is 0-indexed, so channel 1 is array index 0
 */
int* getPWMOutputs();

/**
 * Returns 8-bit bit mask indicating the status of each channel. A 0 means that the channel
 * is functioning (connected or disabled), whilst a 1 means a channel is disconnected
 * (only if its enabled). Therefore if the status is equal to 0, you can assume all
 * the channels are working fine. If its greater than 0, some of the channels have disconnected.
 * Most importantly, if the status equals 0xFF, all the channels have disconnected,
 * which indicates that the UHF connection was lost.
 */
unsigned char getPWMInputStatus(void);

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