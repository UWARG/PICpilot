/**
 * @file SPI.h
 * @author Ian Frosst
 * @date March 13, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include <p33FJ256GP710A.h>
#include "SPI.h"
#include "../clock.h"


/*
 * Use this file as a modular way to interface with a device over SPI.
 */

#define 
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

void initSPI(uint8_t interface, uint16_t clock, spi_mode mode){
    
    if (interface == 1) { // Usually DMA
        SPI1STATbits.SPIEN = 0; // Disable for configuration
                
        IFS0bits.SPI1IF = 0; //Clear interrupt flag
        IEC0bits.SPI1IE = 0; //Disable interrupt (so it doesnt mess with this initialization)
        
        SPI1BUF = 0; // clear buffer

        SPI1STATbits.SPISIDL = 0; //Continue module operation in idle mode // was 1?
        
        SPI1CON1bits.DISSCK = 0; //SPI clock controlled by this module
        SPI1CON1bits.DISSDO = 0; //Output pins are controlled by this module

        SPI1CON1bits.MODE16 = 0; //16/8 bit communication mode (1/0)
        SPI1CON2bits.FRMEN = 0; // Framed mode disabled

        if (mode == SPI_MASTER) {
            SPI1CON1bits.MSTEN = 1; // Master mode
            SPI1CON1bits.SSEN = 0; // clear SS mode
            TRISBbits.TRISB2 = 0; // set SS pin as output
            
            // clock pre-scale bits // TODO: enable dynamic 
            SPI1CON1bits.PPRE = 0b10; // Primary: 4:1 
            SPI1CON1bits.SPRE = 0b100; // Secondary: 4:1 
        } else if (mode == SPI_SLAVE) {
            SPI1CON1bits.MSTEN = 0; // Slave mode
            SPI1CON1bits.SSEN = 0; // disable SS?
            // no need to set clock in slave mode
        }
        
        // clock/sampling bits
        SPI1CON1bits.SMP = 0; //Sample the input at the end of the square wave
        SPI1CON1bits.CKE = 0; //Output data changes from idle state to active clock state (1 is the opposite)
        SPI1CON1bits.CKP = 0; //Idle clock state is low, active clock state is high

        IPC2bits.SPI1IP = 4; // Set interrupt priority
        IEC0bits.SPI1IE = 1; //Enable interrupt

        //Enable SPI
        SPI1STATbits.SPIEN = 1;

    } else if (interface == 2) { // Usually IMU
        
        SPI2STATbits.SPIEN = 0; // Disable for configuration
        SPI2STATbits.SPISIDL = 0; //Continue module operation in idle mode

        SPI2BUF = 0; // clear buffer
        
        IFS2bits.SPI2IF = 0; //Clear interrupt flag
        IEC2bits.SPI2IE = 0; //Disable interrupt (so it doesn't mess with initialization)
    
        SPI2CON1bits.DISSCK = 0; //SPI clock controlled by this module
        SPI2CON1bits.DISSDO = 0; //Output pins are controlled by this module
        
        SPI2CON1bits.MODE16 = 0; //16/8 bit communication mode (1/0)
         
        SPI2CON1bits.MSTEN = mode; // set master or slave mode
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
        SPI2STATbits.SPIEN = 1; //Enable SPI


        TRISGbits.TRISG9 = !mode; // configure SS pin as input or output
    }

}
void sendData(char command){

}

