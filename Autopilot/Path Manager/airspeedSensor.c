/*
 * File:   airspeedSensor.c
 * Author: Chris
 *
 * Created on June 15, 2013, 3:40 PM
 */
#include "main.h"
#include "airspeedSensor.h"


char airspeed = 0;
int currentAirspeedSignal = 0;

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void)
{

    currentAirspeedSignal = ADC1BUF0;
    IFS0bits.AD1IF = 0;		// Clear the ADC2 Interrupt Flag


}

void initAirspeedSensor(){
    //RA6/AN22 is the pin to get the airspeed information
    TRISAbits.TRISA6 = 1;
    initAirspeedADC();

}


char getCurrentAirspeed(){
    airspeed = (char)((long int)currentAirspeedSignal*100/4096);
    return airspeed;
}

void initAirspeedADC(){
    AD1CON1bits.FORM = 0;		// Data Output Format: Unsigned Int
    AD1CON1bits.SSRC = 7;		// Internal Counter (SAMC) ends sampling and starts convertion
    AD1CON1bits.ASAM = 1;		// Sampling begins when SAMP bit is set (for now)
    AD1CON1bits.SIMSAM = 0;		// Sequencial Sampling/conversion
    AD1CON1bits.AD12B = 1;		// 12-bit 2/4-channel operation
    AD1CON1bits.ADDMABM = 0; 		//// DMA buffers are built in conversion order mode
    AD1CON1bits.SAMP = 1;

    AD1CON2bits.SMPI = 0;			// Interrupt address every sample/conversion
    AD1CON2bits.BUFM = 0;
    AD1CON2bits.CHPS = 0;               //Converts channel 0
    AD1CON2bits.VCFG = 0;               //Voltage Reference is 3.3V and Ground Reference is Ground

    AD1CON3bits.ADRC=0;			// ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC=0; 		// Auto Sample Time = 0*Tad
    AD1CON3bits.ADCS=6;			// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*7 = 175nS

    AD1CHS0bits.CH0SA = 22; //Channel 0 positive input on AN22 (Sample A)
    AD1CHS0bits.CH0SB = 22; //Channel 0 positive input on AN22 (Sample B)

    AD1PCFGLbits.PCFG12 = 0; //Port pin set to analog mode (voltage sampling)


    IFS0bits.AD1IF = 0;			// Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 1;			// Enable A/D interrupt
    AD1CON1bits.ADON = 1;		// Turn on the A/D converter

}
