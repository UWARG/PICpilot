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
    IFS0bits.AD1IF = 0;		// Clear the ADC1 Interrupt Flag

}


void initAirspeedSensor(){
    //RA6/AN22 is the pin to get the airspeed information
    TRISAbits.TRISA6 = 1;
    initAirspeedADC();

}


float getCurrentAirspeed(){
      //airspeed = (char)((long int)currentAirspeedSignal*100/4096);
      airspeed = ((float)(pow(currentAirspeedSignal,2)*0.0147+0.4132*currentAirspeedSignal+1743)) //sensor signal calibration
      return currentAirspeedSignal;
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

    AD1PCFGL = 0;
    // TODO why is this an error? AD2PCFGH = 0;
    AD1PCFGLbits.PCFG0 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG1 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG2 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG3 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG4 = 0; //Port pin set to analog mode (voltage sampling)AD2PCFGLbits.PCFG12 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG5 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG6 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG7 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG8 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG9 = 0; //Port pin set to analog mode (voltage sampling)AD2PCFGLbits.PCFG12 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG10 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG11 = 0; //Port pin set to analog mode (voltage sampling)
    AD1PCFGLbits.PCFG12 = 0; //Port pin set to analog mode (voltage sampling)
    
    

    
    IFS0bits.AD1IF = 0;			// Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 1;			// Enable A/D interrupt
    AD1CON1bits.ADON = 1;		// Turn on the A/D converter

}
