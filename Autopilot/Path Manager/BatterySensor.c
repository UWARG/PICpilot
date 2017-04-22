/*
 * File:   voltageSensor.c
 * Author: Chris
 *
 * Created on June 15, 2013, 3:40 PM
 */
#include "main.h"
#include "../Common/Common.h"
#include "BatterySensor.h"
#include "../Common/Utilities/Logger.h"

#define V_SAMPLE_COUNT 50

#define ANALOG_RATIO (3.3f / 4096)

// ratios for voltage dividers
#define MAIN_BATT_RATIO 0.231f
#define EXT_BATT_RATIO 0.138f // NOTE: this is not the actual VD value. gives good voltage, though
#define SERVO_RATIO 0.0f // servo pin VD is sketchy, plus we don't use it

enum ADCPin {
    MAIN_BATT_PIN = 12, // Voltage from main (PICpilot) battery
    EXT_BATT_PIN = 10,  // Voltage from external battery
    SERVO_PIN = 13      // Voltage supply for PWM I/O ports
};

// order for state 
static const uint8_t adc_state[3] = {
    MAIN_BATT_PIN,
    EXT_BATT_PIN,
    //SERVO_PIN
};

static uint8_t cur_state = 0;

static uint32_t current_sum = 0; // current sum of samples
static uint8_t num_samples = 0;

uint16_t main_battery_adc = 0;
uint16_t ext_battery_adc = 0;
uint16_t servo_adc = 0;

void __attribute__((interrupt, no_auto_psv)) _ADC2Interrupt(void){
    uint16_t input = ADC2BUF0;
    current_sum += input;
    num_samples++;
    if (num_samples >= V_SAMPLE_COUNT) {
        switch (adc_state[cur_state]) {
            case MAIN_BATT_PIN:
                main_battery_adc = current_sum / num_samples;
                break;
            case EXT_BATT_PIN:
                ext_battery_adc = current_sum / num_samples;
                break;
            case SERVO_PIN:
                servo_adc = current_sum / num_samples;
                break;
        }
        
        current_sum = 0;
        num_samples = 0;
        
        cur_state++;
        cur_state %= 3; // increment & loop state counter
        AD2CON1bits.ADON = 0; // disable the ADC
        AD2CHS0bits.CH0SA = adc_state[cur_state]; // change the sample sources
        AD2CHS0bits.CH0SB = adc_state[cur_state];
        AD2CON1bits.ADON = 1; // re-enable the ADC
    }
    IFS1bits.AD2IF = 0;		// Clear the ADC Interrupt Flag
}

void initBatteryADC() {
    AD2CON1bits.FORM = 0;	 	// Data Output Format: Unsigned integer
    AD2CON1bits.SSRC = 7;		// Internal Counter (SAMC) ends sampling and starts conversion
    AD2CON1bits.ASAM = 1;		// Sampling begins automatically
    AD2CON1bits.SIMSAM = 0;		// Sequential Sampling/conversion
    AD2CON1bits.AD12B = 1;		// 12-bit, 1-channel operation
    AD2CON1bits.ADDMABM = 0; 	// DMA buffers are built in conversion order mode
    AD2CON1bits.SAMP = 1;       // Enable ADC sampling

    AD2CON2bits.SMPI = 0;		// Interrupt address every sample/conversion
    AD2CON2bits.BUFM = 0;
    AD2CON2bits.CHPS = 0;       // Converts channel 0
    AD2CON2bits.VCFG = 0;       // Voltage Reference is 3.3V and Ground Reference is Ground

    AD2CON3bits.ADRC=0;			// ADC Clock is derived from Systems Clock
    AD2CON3bits.SAMC=0; 		// Auto Sample Time = 0*Tad
    AD2CON3bits.ADCS=6;			// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*7 = 175nS

    AD2CHS0bits.CH0SA = 12;     // Channel 0 positive input on AN12 (Sample A)
    AD2CHS0bits.CH0SB = 12;     // Channel 0 positive input on AN12 (Sample B)

    AD2PCFGL = 0;
    
    IFS1bits.AD2IF = 0;			// Clear the A/D interrupt flag bit
    IEC1bits.AD2IE = 1;			// Enable A/D interrupt
    AD2CON1bits.ADON = 1;		// Turn on the A/D converter
}

void initBatterySensor() {
    //AN10, AN12 and AN13 are the external, main battery, and servo voltage pins, respectively
    TRISBbits.TRISB10 = 1;
    TRISBbits.TRISB12 = 1;
    TRISBbits.TRISB13 = 1;
    initBatteryADC();
}

uint16_t getMainBatteryLevel() {
    // return voltage value * 100
    return (uint16_t)((100.f * (float)main_battery_adc) * (ANALOG_RATIO / MAIN_BATT_RATIO));
}

uint16_t getExtBatteryLevel() {
    return (uint16_t)((100.f * (float)ext_battery_adc) * (ANALOG_RATIO / EXT_BATT_RATIO)) - 50; // fuck it
}

