/*
 * File:   voltageSensor.c
 * Author: Chris
 *
 * Created on June 15, 2013, 3:40 PM
 */
#include "main.h"
#include "voltageSensor.h"
#include "UART2.h"


char lastPercent = 0;
char currentPercent = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void){
    //Timer Interrupt used for checking battery level
    UART2_SendChar(checkBattery());
    IFS0bits.T2IF = 0;
}

void initBatterySensor(){
    //AN12 is the pin to get the battery information
    initADC();
    initTimer();

}

float timeRemaining(){
    float dP = (currentPercent - lastPercent)/vTimeInterval; //Rate of change (Percent/second)
    float time = currentPercent/dP;
    return time;
}

char getCurrentPercent(){
    return currentPercent;
}

char initTimer(){
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
    TMR2 = 0x00; // Clear timer register
    PR2 = 65535; //4462ms //470 * 40; // Load the period value = 1280ms                      //
    IPC1bits.T2IP = 0x01; // Set Timer 2 Interrupt Priority Level - Lowest
    IFS0bits.T2IF = 0; // Clear Timer 2 Interrupt Flag
    IEC0bits.T2IE = 1; // Enable Timer 2 interrupt
    T2CONbits.TON = 1; // Start Timer
}

char checkBattery(){
    lastPercent = currentPercent;
    currentPercent = readADC()/4096;

    return currentPercent;

}

void initADC(){
    AD1CON1bits.AD12B = 1; //12-bit mode
    AD1CON2bits.VCFG = 0; //Voltage Reference is 3.3V and Ground Reference is Ground
    AD1CON3bits.ADCS = 0b00000000; //Analog Conversion Clock (Tcy multiplier) - 1x
    AD1PCFGLbits.PCFG12 = 0; //Port pin set to analog mode (voltage sampling)

    AD1CHS0bits.CH0SA = 0xB; //Channel 0 positive input on AN12 (Sample A)
    AD1CHS0bits.CH0SB = 0xB; //Channel 0 positive input on AN12 (Sample B)
    
    AD1CON2bits.CHPS = 0; //Converts channel 0
    AD1CON1bits.SIMSAM = 0 //Sample channels individually
    AD1CSSLbits.CSS12 = 1; // Enable channel 12 for input scan

    AD1CON2bits.CHPS = 0; //Converts channel 0
    AD1CON2bits.CSCNA = 1; //Scan inputs for channel A bit

    AD1CON1bits.FORM = 0; //Integer Form: 0000 dddd dddd dddd
    AD1CON1bits.ADON = 1;

}

int readADC(){
    return ADC1BUF0;
}