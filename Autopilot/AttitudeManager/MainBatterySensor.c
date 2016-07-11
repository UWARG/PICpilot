/*
 * File:   voltageSensor.c
 * Author: Chris
 *
 * Created on June 15, 2013, 3:40 PM
 */
#include "main.h"
#include "../Common/Common.h"
#include "MainBatterySensor.h"


char percent = 0;
int batteryVoltage = 0;

void __attribute__((interrupt, no_auto_psv)) _ADC2Interrupt(void)
{

    batteryVoltage = ADC2BUF0;
        
    IFS1bits.AD2IF = 0;		// Clear the ADC Interrupt Flag


}

void initMainBatterySensor(){
    //AN12 is the pin to get the battery information
    TRISBbits.TRISB12 = 1;
    TRISBbits.TRISB10 = 1;
    initMainBatteryADC();

}

//float timeRemaining(){
//    float dP = (currentPercent - lastPercent)/vTimeInterval; //Rate of change (Percent/second)
//    float time = currentPercent/dP;
//    return time;
//}

int getMainBatteryLevel(){
    //return value is (voltage*10-64), to stay in the range of a char
    //return ((double)(batteryVoltage)/3/127*13)-64;
    return ((double)(batteryVoltage)/3/100*13);
}

void initMainBatteryADC(){
    AD2CON1bits.FORM = 0;		// Data Output Format: Unsigned Int
    AD2CON1bits.SSRC = 7;		// Internal Counter (SAMC) ends sampling and starts convertion
    AD2CON1bits.ASAM = 1;		// Sampling begins when SAMP bit is set (for now)
    AD2CON1bits.SIMSAM = 0;		// Sequencial Sampling/conversion
    AD2CON1bits.AD12B = 1;		// 12-bit 2/4-channel operation
    AD2CON1bits.ADDMABM = 0; 		//// DMA buffers are built in conversion order mode
    AD2CON1bits.SAMP = 1;

    AD2CON2bits.SMPI = 0;			// Interrupt address every sample/conversion
    AD2CON2bits.BUFM = 0;
    AD2CON2bits.CHPS = 0;               //Converts channel 0
    AD2CON2bits.VCFG = 0;               //Voltage Reference is 3.3V and Ground Reference is Ground

    AD2CON3bits.ADRC=0;			// ADC Clock is derived from Systems Clock
    AD2CON3bits.SAMC=0; 		// Auto Sample Time = 0*Tad
    AD2CON3bits.ADCS=6;			// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*7 = 175nS

    AD2CHS0bits.CH0SA = 12; //Channel 0 positive input on AN12 (Sample A)
    AD2CHS0bits.CH0SB = 12; //Channel 0 positive input on AN12 (Sample B)

    AD2PCFGL = 0;
    AD2PCFGLbits.PCFG0 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG1 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG2 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG3 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG4 = 0; //Port pin set to analog mode (voltage sampling)AD2PCFGLbits.PCFG12 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG5 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG6 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG7 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG8 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG9 = 0; //Port pin set to analog mode (voltage sampling)AD2PCFGLbits.PCFG12 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG10 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG11 = 0; //Port pin set to analog mode (voltage sampling)
    AD2PCFGLbits.PCFG12 = 0; //Port pin set to analog mode (voltage sampling)
    
    

    
    IFS1bits.AD2IF = 0;			// Clear the A/D interrupt flag bit
    IEC1bits.AD2IE = 1;			// Enable A/D interrupt
    AD2CON1bits.ADON = 1;		// Turn on the A/D converter

}
