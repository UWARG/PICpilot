/*
 * File:   airspeedSensor.c
 * Author: Chris
 *
 * Created on June 15, 2013, 3:40 PM
 */
#include "main.h"
#include "../Common/Common.h"
#include "airspeedSensor.h"
#include "../Common/Utilities/Logger.h"

// Internal analog reference voltage
#define AREF 3.3f
// Source voltage for the airspeed sensor
#define VSOURCE 4.0f
// Voltage divider ratio
#define ASPD_RATIO 0.677f

const static float v_sc = ((AREF / 4096) * ASPD_RATIO) / VSOURCE; // ADC -> voltage -> percentage

static int airspeedHistory[AIRSPEED_HISTORY] = {0};
static int historyCounter = 0;

static float offset = 0;

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void) {
    airspeedHistory[historyCounter++] = ADC1BUF0;
    historyCounter %= AIRSPEED_HISTORY;
    IFS0bits.AD1IF = 0; // Clear the ADC1 Interrupt Flag
}

// run on startup to get a zero value for airspeed. 
// waits until the airspeed buffer is fully populated, then saves the offset 
void calibrateAirspeed() {
    int i;
    bool done = false;
    while (!done) {
        offset = 0;
        done = true;
        for (i = 0; i < AIRSPEED_HISTORY; i++) {
            if (airspeedHistory[i] == 0.0) {
                done = false;
                break;
            }
            offset += airspeedHistory[i];
        }
    }
    offset /= AIRSPEED_HISTORY;
    debugInt("Airspeed Offset", (int)offset); // hopefully will be close to 0. if this is > 2000, something might be wrong.
}

void initAirspeedSensor(){
    //RA11/AN11 is the pin to get the airspeed information
    TRISBbits.TRISB11 = 1;
    initAirspeedADC();
    //calibrateAirspeed();
}

static float ADCConvert(float signal) {
    // converts an ADC value (0-4095) to an airspeed (m/s)
    return sqrtf(((signal*v_sc) - 0.5f) * 333.33f);
}

float getCurrentAirspeed(){
    int i;
    float aspd_filtered = 0;
    for (i = 0; i < AIRSPEED_HISTORY; i++){
        aspd_filtered += (float)airspeedHistory[i];
    }
    aspd_filtered /= AIRSPEED_HISTORY;
    return ADCConvert(aspd_filtered);
}

void initAirspeedADC(){
    AD1CON1bits.FORM = 0;		// Data Output Format: Unsigned integer
    AD1CON1bits.SSRC = 7;		// Internal Counter (SAMC) ends sampling and starts conversion
    AD1CON1bits.ASAM = 1;		// Sampling begins when SAMP bit is set (for now)
    AD1CON1bits.SIMSAM = 0;		// Sequential Sampling/conversion
    AD1CON1bits.AD12B = 1;		// 12-bit 2/4-channel operation
    AD1CON1bits.ADDMABM = 0;    // DMA buffers are built in conversion order mode
    AD1CON1bits.SAMP = 1;       // Enable ADC sampling

    AD1CON2bits.SMPI = 0;		// Interrupt address every sample/conversion
    AD1CON2bits.BUFM = 0;
    AD1CON2bits.CHPS = 0;       //Converts channel 0
    AD1CON2bits.VCFG = 0;       //Voltage Reference is 3.3V and Ground Reference is Ground

    AD1CON3bits.ADRC=0;			// ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC=0; 		// Auto Sample Time = 0*Tad
    AD1CON3bits.ADCS=6;			// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*7 = 175nS

    AD1CHS0bits.CH0SA = 11;     //Channel 0 positive input on AN11 (Sample A)
    AD1CHS0bits.CH0SB = 11;     //Channel 0 positive input on AN11 (Sample B)

    AD1PCFGL = 0;               // Set all pins to analog input
    AD1PCFGH = 0;

    IFS0bits.AD1IF = 0;			// Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 1;			// Enable A/D interrupt
    AD1CON1bits.ADON = 1;		// Turn on the A/D converter

}
