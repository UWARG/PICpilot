/* 
 * File:   PWMExtension.h
 * Author: Chris Hajduk
 *
 * Created on April 29, 2015, 10:23 AM
 */

#ifndef PWMEXTENSION_H
#define	PWMEXTENSION_H

#ifdef	__cplusplus
extern "C" {
#endif

#define NUM_CHANNELS_EXT 16

//This is defined according to the documentation of the PCA9685 chip
#define UPPER_PWM_EXT 410
#define MIDDLE_PWM_EXT 308
#define LOWER_PWM_EXT 205

#define PWM_DEVICE_ADDRESS 0b1110001
#define PWM_REGISTER_OFFSET 0x06
#define PWM_REGISTER_GROUPING 4
#define PRESCALE_REGISTER 0xFE
#define PRESCALE_VALUE 121 // This is a 50Hz update rate, with a 25MHz clock

// Function Prototypes
/*****************************************************************************
 * Function: void initPWM(int outputChannels);
 *
 * Preconditions: None.
 *
 * Overview: Initializes the PWM ports. For instance if outputChannels = 0b00100101
 *      then channels 1,3, and 6 will be initialized.
 *
 * Input: char outputChannels -> The inputs to be initialized. Each bit represents
 *          a channel.
 *
 * Output: None.
 *
 *****************************************************************************/
void initPWMExtension(int outputChannels);

/*****************************************************************************
 * Function: void PWMExtensionOutputCalibration(unsigned int channel, float signalScaleFactor, unsigned int signalOffset);
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
void PWMExtensionOutputCalibration(unsigned int channel, float signalScaleFactor, unsigned int signalOffset);

/*****************************************************************************
 * Function: void setPWM(unsigned int channel, int pwm);
 *
 * Preconditions: The PWM outputs must have been initialized for valid output.
 *
  Overview: Sets the output through the output compare pins on the on the I2C device.
 *
 * Input: unsigned int channel -> The channel that is being set.
 *        int pwm -> The value (usually between +-1024) that is to be set to the
 *                  output compare pins.
 *
 * Output: None.
 *
 *****************************************************************************/
void setPWMExtension(unsigned int channel, int pwm);

/*****************************************************************************
 * Function: void setPWMArrayExtension(int* ocArray);
 *
 * Preconditions: The PWM outputs must have been initialized for valid output.
 *
  Overview: Sets the output through the output compare pins on the on the I2C device
 *          according to the values indicated in the ocArray.
 *
 * Input: int* ocArray -> An array containing the values for each corresponding
 *                       channel.
 *
 * Output: None.
 *
 *****************************************************************************/
void setPWMArrayExtension(int* ocArray);


#ifdef	__cplusplus
}
#endif

#endif	/* PWMEXTENSION_H */

