/*
 * File:   PWMExtension.c
 * Author: Chris Hajduk
 *
 * Created on April 29, 2015, 10:23 AM
 */

#include "PWM.h"
#include "debug.h"
#include "PWMExtension.h"
#include "I2C.h"

char initializedExtension = 0;

float scaleFactorOutExtension[NUM_CHANNELS_EXT];
int offsetOutExtension[NUM_CHANNELS_EXT];

void initPWMExtension(int outputChannels){
    initI2C();
    //Send prescale value
    char prescale = PRESCALE_VALUE;
    sendMessage(PWM_DEVICE_ADDRESS, PRESCALE_REGISTER, &prescale, 1, WRITE);
    int i = 0;
    for (i = 0; i < NUM_CHANNELS_EXT; i++){
        if (outputChannels & (1 << i)){
            unsigned int PWMOn = 0;
            sendMessage(PWM_DEVICE_ADDRESS, PWM_REGISTER_GROUPING * i + PWM_REGISTER_OFFSET, (char*)(&PWMOn), 2, WRITE);
            unsigned int PWMOff = MIDDLE_PWM_EXT;
            sendMessage(PWM_DEVICE_ADDRESS, PWM_REGISTER_GROUPING * i + PWM_REGISTER_OFFSET, (char*)(&PWMOff), 2, WRITE);
        }
        // Use default scale factor values
        scaleFactorOutExtension[i] = (float)(UPPER_PWM_EXT - MIDDLE_PWM_EXT)/MAX_PWM; // We know that MAX_PWM = (UPPER_PWM - MIDDLE_PWM) for a factor of 1.
        // Use default offset values
        offsetOutExtension[i] = MIDDLE_PWM_EXT;
    }
    //**************initOC(outputChannels);
    initializedExtension = 1;
}

void PWMExtensionOutputCalibration(unsigned int channel, float signalScaleFactor, unsigned int signalOffset){
    if (channel > 0 && channel <= NUM_CHANNELS_EXT){ //Check if channel number is valid
        scaleFactorOutExtension[channel - 1] = signalScaleFactor;
        offsetOutExtension[channel - 1] = signalOffset;
    }
    else { //Invalid Channel
        //Display Error Message
        error("Invalid PWM channel");
    }
}

void setPWMExtension(unsigned int channel, int pwm){
    if (initializedExtension && channel > 0 && channel <= NUM_CHANNELS_EXT){ //Is the Input Initialized?
        unsigned int PWMOff = (int)(pwm * scaleFactorOutExtension[channel - 1] + offsetOutExtension[channel - 1]);
        sendMessage(PWM_DEVICE_ADDRESS, PWM_REGISTER_GROUPING * (channel - 1) + PWM_REGISTER_OFFSET, (char*)(&PWMOff), 2, WRITE);
    }
    else { //Not initialized or invalid channel
        //Display Error Message
        error("PWM not initialized or invalid channel specified");
    }
}
void setPWMArrayExtension(int* ocArray){
    if (initializedExtension){ //Is the Input Initialized?
        int i = 0;
        for (i = 0; i < NUM_CHANNELS_EXT; i++){
            unsigned int PWMOff = (int)(ocArray[i] * scaleFactorOutExtension[i] + offsetOutExtension[i]);
            sendMessage(PWM_DEVICE_ADDRESS, PWM_REGISTER_GROUPING * i + PWM_REGISTER_OFFSET, (char*)(&PWMOff), 2, WRITE);
        }
    }
    else { //Not initialized
        //Display Error Message
        error("PWM not initialized.");
    }
}