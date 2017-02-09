/**
 * @file PWM.c
 * @created August 21, 2014, 11:40 PM
 */
#include "PWM.h"
#include "OutputCompare.h"
#include "InputCapture.h"
#include "timer.h"

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
 * Contains the scaled PWM inputs received from the RC controller from MIN_PWM - MAX_PWM
 */
static int pwm_inputs[NUM_CHANNELS];

/**
 * Contains the pre-scaled PWM outputs that were last set from MIN_PWM - MAX_PWM
 */
static int pwm_outputs[NUM_CHANNELS]; //The output for status updates

/**
 * Used to keep track of the connected/disconnected input channels
 */
static unsigned char disconnected_pwm_inputs;

/**
 * Used so that we dont mark disabled channels as disconnected
 */
static unsigned char enabled_input_channels;

/**
 * Scale factors and offsets for each of the 8 channels
 */
static float input_scale_factors[NUM_CHANNELS];
static int input_offsets[NUM_CHANNELS];
static float output_scale_factors[NUM_CHANNELS];
static int output_offsets[NUM_CHANNELS];

void initPWM(unsigned char inputChannels, unsigned char outputChannels)
{
    initTimer2();
    initIC(inputChannels);
    initOC(outputChannels);

    enabled_input_channels = inputChannels;
    disconnected_pwm_inputs = 0; //mark none of the inputs as disconnected

    //Set the initial offsets and scaling factors
    int i = 0;
    for (i = 0; i < NUM_CHANNELS; i++) {
        input_scale_factors[i] = DEFAULT_INPUT_SCALE_FACTOR;
        output_scale_factors[i] = DEFAULT_OUTPUT_SCALE_FACTOR;
        output_offsets[i] = MIDDLE_PWM;
        input_offsets[i] = MIDDLE_PWM;
    }
}

int* getPWMArray(unsigned long int sys_time)
{
    unsigned char channel_enabled;
    unsigned int* ic_values = getICValues(sys_time);
    int channel = 0;
    
    for (channel = 0; channel < NUM_CHANNELS; channel++) {
        channel_enabled = enabled_input_channels & (1 << channel);
        
        if(!channel_enabled){ //if the channel is disabled
            pwm_inputs[channel] = DISCONNECTED_PWM_VALUE;
        } else if (ic_values[channel] == 0 && channel_enabled) { //if the channel is enabled but disconnected
            pwm_inputs[channel] = DISCONNECTED_PWM_VALUE;
            disconnected_pwm_inputs = disconnected_pwm_inputs | (1 << channel); //set the bit as 1
        } else { //otherwise if its a connected, enabled channel, calculate its value
            pwm_inputs[channel] = (int) ((ic_values[channel] - input_offsets[channel]) * input_scale_factors[channel]);
            disconnected_pwm_inputs = disconnected_pwm_inputs & (~(1 << channel)); //set the bit as 0
        }
    }
    return pwm_inputs;
}

void setPWM(unsigned int channel, int pwm)
{
    if (channel > 0 && channel <= NUM_CHANNELS && pwm >= MIN_PWM && pwm <= MAX_PWM) {
        pwm_outputs[channel - 1] = pwm;
        setOCValue(channel - 1, (int) (pwm * output_scale_factors[channel - 1] + output_offsets[channel - 1]));
    }
}

int* getPWMOutputs()
{
    return pwm_outputs;
}

unsigned char getPWMInputStatus(void)
{
    return disconnected_pwm_inputs;
}

void calibratePWMInputs(unsigned int channel, float signalScaleFactor, unsigned int signalOffset)
{
    if (channel > 0 && channel <= NUM_CHANNELS) { //Check if channel number is valid
        input_scale_factors[channel - 1] = signalScaleFactor;
        input_offsets[channel - 1] = signalOffset;
    }
}

void calibratePWMOutputs(unsigned int channel, float signalScaleFactor, unsigned int signalOffset)
{
    if (channel > 0 && channel <= NUM_CHANNELS) { //Check if channel number is valid
        output_scale_factors[channel - 1] = signalScaleFactor;
        output_offsets[channel - 1] = signalOffset;
    }
}