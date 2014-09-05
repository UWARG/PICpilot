/* 
 * File:   PWM.h
 * Author: Chris Hajduk
 *
 * Created on August 21, 2014, 11:45 PM
 */

#ifndef PWM_H
#define	PWM_H

#ifdef	__cplusplus
extern "C" {
#endif

// Includes
#include "main.h"

// Definitions
#define NUM_CHANNELS 2
#define MAX_PWM 1024
#define MIN_PWM -1024

// Function Prototypes
/*****************************************************************************
 * Function: void initPWM(char inputChannels, char outputChannels);
 *
 * Preconditions: None.
 *
 * Overview: Initializes the PWM ports. For instance if inputChannels = 0b00100101
 *      then channels 1,3, and 6 will be initialized.
 *
 * Input: char inputChannels -> The inputs to be initialized. Each bit represents
 *          a channel.
 *
 * Output: None.
 *
 *****************************************************************************/
void initPWM(char inputChannels, char outputChannels);

/*****************************************************************************
 * Function: void PWMInputCalibration(unsigned int channel, float signalScaleFactor, unsigned int signalOffset);
 *
 * Preconditions: None.
 *
 * Overview: Calibrates the input range and trim of the device.
 *
 * Input: unsigned int channel -> The channel to calibrate.
 *        float signalScaleFactor -> The scale factor to increase or decrease the range by.
 *                                  For instance, if the target range is from -1024 to 1024,
 *                                  the scale factor would be 1024/(half the detected input range)
 *        unsigned int signalOffset -> Compensation for a non-zero offset on the input. This is
 *                                      known as trim.
 *
 * Output: None.
 *
 *****************************************************************************/
void PWMInputCalibration(unsigned int channel, float signalScaleFactor, unsigned int signalOffset);

/*****************************************************************************
 * Function: void PWMdOutputCalibration(unsigned int channel, float signalScaleFactor, unsigned int signalOffset);
 *
 * Preconditions: None.
 *
 * Overview: Calibrates the output range and trim of the device.
 *
 * Input: unsigned int channel -> The channel to calibrate.
 *        float signalScaleFactor -> The scale factor to increase or decrease the range by.
 *                                  For instance, if the range is from -1024 to 1024,
 *                                  the scale factor would be (half the desired output range)/1024
 *        unsigned int signalOffset -> Compensation for a non-zero offset on the input. This is
 *                                      known as trim.
 *
 * Output: None.
 *
 *****************************************************************************/
void PWMOutputCalibration(unsigned int channel, float signalScaleFactor, unsigned int signalOffset);

/*****************************************************************************
 * Function: int getPWM(unsigned int channel);
 *
 * Preconditions: The PWM inputs must have been initialized for valid data.
 *
 * Overview: Retrieves input (via controller) through the input capture pins on
 *             the device.
 *
 * Input: unsigned int channel -> The channel to collect the data from.
 *
 * Output: int -> The input signal of the channel. Determined by the scaling
 *                factor range (usually 1024).
 *
 *****************************************************************************/
int getPWM(unsigned int channel);

/*****************************************************************************
 * Function: int* getPWMArray();
 *
 * Preconditions: The PWM inputs must have been initialized for valid data.
 *
 * Overview: Retrieves input (via controller) through the input capture pins on
 *             the device.
 *
 * Input: None.
 *
 * Output: int* -> A pointer to the values of the array, which represent the
 *                  inputs of each channel. (usuall ranging between +-1024)
 *
 *****************************************************************************/
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
void setPWM(unsigned int channel, int pwm);

/*****************************************************************************
 * Function: void setPWMArray(int* ocArray);
 *
 * Preconditions: The PWM outputs must have been initialized for valid output.
 *
  Overview: Sets the output through the output compare pins on the device
 *          according to the values indicated in the ocArray.
 *
 * Input: int* ocArray -> An array containing the values for each corresponding
 *                       channel.
 *
 * Output: None.
 *
 *****************************************************************************/
void setPWMArray(int* ocArray);

#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

