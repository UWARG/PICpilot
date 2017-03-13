/**
 * @file SPI.h
 * @author Ian Frosst
 * @date March 13, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include <p33FJ256GP710.h>
#include "SPI.h"
#include "../clock.h"

#include <stdint.h>


/*
 * Use this file as a modular way to interface with a device over SPI.
 */

typedef enum {
    PIN_LOW = 0,
    PIN_HIGH
} pin_state;

/*
 * SPI uses 4 pins:
 * SDIx: Serial Data Input              --> 1 : RF7, 2 : RG7
 * SDOx: Serial Data Output             --> 1 : RF8, 2 : RG8
 * SCKx: Shift Clock Input/Output       --> 1 : RF6, 2 : RG6
 * SSx/FSYNCx: Slave select/Frame Sync  --> 1 : RB2, 2 : RG9
 * Slave select is active-low
 */

void SPI_SetSS(uint8_t interface, pin_state state) {
    if (interface == 1) {
        PORTBbits.RB2 = state;
    } else if (interface == 2) {
        PORTGbits.RG9 = state;
    }
}

void initSPI(uint8_t interface, uint16_t clock, uint8_t master){
    
    if (interface == 1) {

        //Set interrupts
        IFS0bits.SPI1IF = 0; //Clear interrupt flag
        IEC0bits.SPI1IE = 0; //Disable interrupt (so it doesnt mess with this initialization)

        //Continue module operation in idle mode
        SPI1STATbits.SPISIDL = 0;
        //SPI clock controlled by this module
        SPI1CON1bits.DISSCK = 0;
        //Output pins are controlled by this module
        SPI1CON1bits.DISSDO = 0;
        //16/8 bit communication mode (1/0)
        SPI1CON1bits.MODE16 = 0; //8
        //Master mode(1)/Slave mode(0)
        SPI1CON1bits.MSTEN = master; //MASTER
        //Sample Phase (end/middle)
        SPI1CON1bits.SMP = 0; //Sample the input at the end of the square wave
        //Clock Edge Select
        SPI1CON1bits.CKE = 0; //Output data changes from idle state to active clock state (1 is the opposite)
        //Clock Polarity
        SPI1CON1bits.CKP = 0; //Idle clock state is low, active clock state is high
        //Secondary Prescale (The prescale of the prescale)(3 bits)
        SPI1CON1bits.SPRE = 0b000; //8:1 prescale
        //Primart Prescale (The prescale of the clock) (2 bits)
        SPI1CON1bits.PPRE = 0b00; //64:1 prescale

        //Then enable interrupts
        IFS0bits.SPI1IF = 0; //Clear interrupt flag
        IEC0bits.SPI1IE = 1; //Enable interrupt

        //Enable SPI
        SPI1STATbits.SPIEN = 1;
        
        TRISBbits.TRISB2 = !master;

        //Then write to the SPI1BUF
    } else if (interface == 2) {
        
        SPI2STATbits.SPIEN = 0; // Disable for configuration
        SPI2STATbits.SPISIDL = 0; //Continue module operation in idle mode

        IFS2bits.SPI2IF = 0; //Clear interrupt flag
        IEC2bits.SPI2IE = 0; //Disable interrupt (so it doesn't mess with initialization)
    
        SPI2CON1bits.DISSCK = 0; //SPI clock controlled by this module
        SPI2CON1bits.DISSDO = 0; //Output pins are controlled by this module
        
        SPI2CON1bits.MODE16 = 0; //16/8 bit communication mode (1/0)
        
        SPI2CON1bits.MSTEN = master; // set master or slave mode
        //Sample Phase (end/middle)
        SPI2CON1bits.SMP = 0; //Sample the input at the middle of the square wave
        //Clock Edge Select
        SPI2CON1bits.CKE = 0; //Output data changes from idle state to active clock state (1 is the opposite)
        //Clock Polarity
        SPI2CON1bits.CKP = 1; //Idle clock state is high, active clock state is low
        //Secondary Prescale (The prescale of the prescale)(3 bits)
        SPI2CON1bits.SPRE = 0b110; //2:1 prescale
        //Primary Prescale (The prescale of the clock) (2 bits)
        SPI2CON1bits.PPRE = 0b10; //4:1 prescale

        //Then enable interrupts
        IFS2bits.SPI2IF = 0; //Clear interrupt flag
        IEC2bits.SPI2IE = 0; //Disable interrupt

        SPI2STATbits.SPIROV = 1;
        //Enable SPI
        SPI2STATbits.SPIEN = 1;

        
        TRISGbits.TRISG9 = !master;
    }

}
void sendData(char command){

}

