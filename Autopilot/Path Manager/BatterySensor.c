/*
 * File:   voltageSensor.c
 * Author: Chris
 *
 * Created on June 15, 2013, 3:40 PM
 */
#include "main.h"
#include "../Common/Common.h"
#include "BatterySensor.h"

int main_battery = 0;
int ext_battery = 0;


void __attribute__((interrupt, no_auto_psv)) _ADC2Interrupt(void){
    main_battery = ADC2BUF0;
    IFS1bits.AD2IF = 0;		// Clear the ADC Interrupt Flag
}

static void initBatteryADC(){
    AD2CON1bits.FORM = 0;		// Data Output Format: Unsigned integer
    AD2CON1bits.SSRC = 7;		// Internal Counter (SAMC) ends sampling and starts conversion
    AD2CON1bits.ASAM = 1;		// Sampling begins automatically
    AD2CON1bits.SIMSAM = 0;		// Sequential Sampling/conversion
    AD2CON1bits.AD12B = 0;		// 10-bit, 4-channel operation
    AD2CON1bits.ADDMABM = 0; 	// DMA buffers are built in conversion order mode

    AD2CON2bits.SMPI = 0;		// Interrupt address every sample/conversion
    AD2CON2bits.BUFM = 0;
    AD2CON2bits.CHPS = 1;       // Converts channel 0 & channel 1
    AD2CON2bits.VCFG = 0;       // Voltage Reference is 3.3V and Ground Reference is Ground

    AD2CON3bits.ADRC=0;			// ADC Clock is derived from Systems Clock
    AD2CON3bits.SAMC=0; 		// Auto Sample Time = 0*Tad
    AD2CON3bits.ADCS=6;			// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*7 = 175nS

    AD2CHS0bits.CH0SA = 12; //Channel 0 positive input on AN12 (Sample A)
    AD2CHS0bits.CH0SB = 12; //Channel 0 positive input on AN12 (Sample B)

    AD2PCFGL = 0;
    
    IFS1bits.AD2IF = 0;			// Clear the A/D interrupt flag bit
    IEC1bits.AD2IE = 1;			// Enable A/D interrupt
    AD2CON1bits.ADON = 1;		// Turn on the A/D converter

}

void initMainBatterySensor(){
    //AN12 is the pin to get the battery information
    TRISBbits.TRISB12 = 1;
    initMainBatteryADC();
}

int getMainBatteryLevel(){
//return raw voltage value
    return batteryVoltage;
}

int getExtBatteryLevel() {
    
}