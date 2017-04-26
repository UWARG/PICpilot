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

// Number of ADC readings to average
#define AIRSPEED_HISTORY 20

// Internal analog reference voltage ratio
#define AREF_RATIO (3.3f / 4096)
// Source voltage for the airspeed sensor
#define VSOURCE 5.0f
// Voltage divider ratio
#define ASPD_RATIO 0.677f

// ADC -> voltage -> percentage -> pressure, then multiplied by some factors for Bernoulli
static float v_sc = ((AREF_RATIO / ASPD_RATIO) / VSOURCE) * 0.2f  * 1666.67f; // derived from the internal ADC conversions and the airspeed sensor datasheet
static float ardu = 1.9936f; // magic number via ardupilot.

static int airspeedHistory[AIRSPEED_HISTORY] = {0};
static int historyCounter = 0;

static float offset = 0;

static bool new_data = false;
static float last_speed = 0;

static void initAirspeedADC(){
    AD1CON1bits.FORM = 0;		// Data Output Format: Unsigned integer
    AD1CON1bits.SSRC = 7;		// Internal Counter (SAMC) ends sampling and starts conversion
    AD1CON1bits.ASAM = 1;		// Sampling begins when SAMP bit is set (for now)
    AD1CON1bits.SIMSAM = 0;		// Sequential Sampling/conversion
    AD1CON1bits.AD12B = 1;		// 12-bit 2/4-channel operation
    AD1CON1bits.ADDMABM = 0;    // DMA buffers are built in conversion order mode
    AD1CON1bits.SAMP = 1;       // Enable ADC sampling

    AD1CON2bits.SMPI = 0;		// Interrupt address every sample/conversion
    AD1CON2bits.BUFM = 0;
    AD1CON2bits.CHPS = 0;       // Converts channel 0
    AD1CON2bits.VCFG = 0;       // Voltage Reference is 3.3V and Ground Reference is Ground

    AD1CON3bits.ADRC=0;			// ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC=0; 		// Auto Sample Time = 0*Tad
    AD1CON3bits.ADCS=6;			// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*7 = 175nS

    AD1CHS0bits.CH0SA = 11;     // Channel 0 positive input on AN11 (Sample A)
    AD1CHS0bits.CH0SB = 11;     // Channel 0 positive input on AN11 (Sample B)

    AD1PCFGL = 0;               // Set all pins to analog input
    AD1PCFGH = 0;

    IFS0bits.AD1IF = 0;			// Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 1;			// Enable A/D interrupt
    AD1CON1bits.ADON = 1;		// Turn on the A/D converter
}

/**
 * Converts an ADC signal (0-4095) to an airspeed (m/s).
 * Derived from the equation for flow velocity given impact pressure (for fluids)
 * @param signal Signal from ADC, in 0-4095
 * @return Airspeed, in m/s
 */
static float ADCConvert(uint16_t signal) {
    return sqrtf(signal * v_sc);
}

// run on startup to get a zero value for airspeed. 
// waits until the airspeed buffer is fully populated, then saves the offset 
void calibrateAirspeed() {
    int i, j;
    bool done = false;
    for (j = 0; j < 256; j++) {
        if (done) break;
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
    debugInt("Airspeed Offset", (int32_t)offset); // should be pretty close to 2000 (+/- 200)
}

void initAirspeedSensor(){
    //RA11/AN11 is the pin to get the airspeed information
    TRISBbits.TRISB11 = 1;
    initAirspeedADC();
    calibrateAirspeed();
}

float getCurrentAirspeed(){
    if (new_data) {
        new_data = false;
        int i;
        uint16_t aspd_filtered = 0;
        for (i = 0; i < AIRSPEED_HISTORY; i++){
            aspd_filtered += airspeedHistory[i];
        }
        aspd_filtered /= AIRSPEED_HISTORY;
        last_speed = ADCConvert(abs(aspd_filtered - offset));
    }
    return last_speed;
}

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void) {
    new_data = true;
    airspeedHistory[historyCounter++] = ADC1BUF0;
    historyCounter %= AIRSPEED_HISTORY;
    IFS0bits.AD1IF = 0; // Clear the ADC1 Interrupt Flag
}
