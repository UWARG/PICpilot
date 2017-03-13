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

/*
 * SPI uses 4 pins:
 * SDIx: Serial Data Input              --> 1 : RF7, 2 : RG7
 * SDOx: Serial Data Output             --> 1 : RF8, 2 : RG8
 * SCKx: Shift Clock Input/Output       --> 1 : RF6, 2 : RG6
 * SSx/FSYNCx: Slave select/Frame Sync  --> 1 : RB2, 2 : RG9
 * Slave select is active-low
 */

void SPI_SS(uint8_t interface, pin_state state) {
    if (interface == 1) {
        PORTBbits.RB2 = state;
    } else if (interface == 2) {
        PORTGbits.RG9 = state;
    }
}

void initSPI(uint8_t interface, uint16_t clock, spi_mode mode, spi_width width, spi_type mss){

    if (interface == 1) { // Usually DMA
        SPI1STATbits.SPIEN = 0; // Disable for configuration
                
        IFS0bits.SPI1IF = 0; //Clear interrupt flag
        IEC0bits.SPI1IE = 0; //Disable interrupt (so it doesnt mess with this initialization)
        
        SPI1BUF = 0; // clear buffer

        SPI1STATbits.SPISIDL = 0; //Continue module operation in idle mode // was 1?
        SPI1STATbits.SPIROV = 0; // Clear receive overflow
        SPI1CON1bits.DISSCK = 0; //SPI clock controlled by this module
        SPI1CON1bits.DISSDO = 0; //Output pins are controlled by this module

        SPI1CON1bits.MODE16 = width; //16/8 bit communication mode (1/0)
        SPI1CON2bits.FRMEN = 0; // Framed mode disabled

        if (mss == SPI_MASTER) {
            SPI1CON1bits.MSTEN = 1; // Master mode
            SPI1CON1bits.SMP = 1; // sample input ad end of wave
            SPI1CON1bits.SSEN = 0; // clear SS mode    
            TRISBbits.TRISB2 = 0; // set SS pin as output
            
            // clock pre-scale bits: 2500 kHz // TODO: enable dynamic scaling
            SPI1CON1bits.PPRE = 0b10; // Primary: 4:1 
            SPI1CON1bits.SPRE = 0b100; // Secondary: 4:1 
        } else if (mss == SPI_SLAVE) {
            SPI1CON1bits.MSTEN = 0; // Slave mode
            SPI1CON1bits.SMP = 0; // sample input in middle of wave
            SPI1CON1bits.SSEN = 0; // disable SS?
            // no need to set clock in slave mode
        }
        
        switch (mode) {
            case SPI_MODE0:
                SPI1CON1bits.CKE = 1; 
                SPI1CON1bits.CKP = 0;
                break;
            case SPI_MODE1:
                SPI1CON1bits.CKE = 0; 
                SPI1CON1bits.CKP = 0;
                break;
            case SPI_MODE2:
                SPI1CON1bits.CKE = 1; 
                SPI1CON1bits.CKP = 1;
                break;
            case SPI_MODE3:
                SPI1CON1bits.CKE = 0; 
                SPI1CON1bits.CKP = 1;
                break;
        }

        // TODO: see if we need SPI interrupts for DMA?
        IPC2bits.SPI1IP = 4; // Set interrupt priority
        IEC0bits.SPI1IE = 1; //Enable interrupt

        SPI1STATbits.SPIEN = 1; //Enable SPI

    } else if (interface == 2) { // Usually IMU / GPS
        SPI2STATbits.SPIEN = 0; // Disable for configuration
        
        IFS2bits.SPI2IF = 0; //Clear interrupt flag
        IEC2bits.SPI2IE = 0; //Disable interrupt (so it doesn't mess with initialization)
        
        SPI2BUF = 0; // clear buffer
        
        SPI2STATbits.SPISIDL = 0; //Continue module operation in idle mode
        SPI2STATbits.SPIROV = 0; // Clear receive overflow
        SPI2CON1bits.DISSCK = 0; //SPI clock controlled by this module
        SPI2CON1bits.DISSDO = 0; //Output pins are controlled by this module
        
        SPI2CON1bits.MODE16 = width; //16/8 bit communication mode (1/0)
        SPI2CON2bits.FRMEN = 0; // disable framed mode
        
        if (mss == SPI_MASTER) {
            SPI2CON1bits.MSTEN = 1; // Master mode
            SPI2CON1bits.SMP = 1; // sample input ad end of wave
            SPI2CON1bits.SSEN = 0; // clear SS mode            
            TRISGbits.TRISG9 = 0; // set SS pin as output

            // clock pre-scale bits: 5000 kHz // TODO: enable dynamic scaling
            SPI2CON1bits.PPRE = 0b10; // Primary: 4:1 
            SPI2CON1bits.SPRE = 0b111; // Secondary: 1:1 
        } else if (mss == SPI_SLAVE) {
            SPI2CON1bits.MSTEN = 0; // Slave mode
            SPI2CON1bits.SMP = 0; // sample input in middle of wave
            SPI2CON1bits.SSEN = 0; // disable SS?
            // no need to set clock in slave mode
        }
        
        switch (mode) {
            case SPI_MODE0:
                SPI2CON1bits.CKE = 1; 
                SPI2CON1bits.CKP = 0;
                break;
            case SPI_MODE1:
                SPI2CON1bits.CKE = 0; 
                SPI2CON1bits.CKP = 0;
                break;
            case SPI_MODE2:
                SPI2CON1bits.CKE = 1; 
                SPI2CON1bits.CKP = 1;
                break;
            case SPI_MODE3:
                SPI2CON1bits.CKE = 0; 
                SPI2CON1bits.CKP = 1;
                break;
        }
        
        IEC2bits.SPI2IE = 0; //Disable interrupt

        SPI2STATbits.SPIEN = 1; //Enable SPI

    }

}
void sendData(char command){

}

