/**
 * @file SPI.c
 * @author Ian Frosst
 * @date March 13, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include <p33FJ256GP710A.h>
#include "SPI.h"
#include "../Clock/Clock.h"

typedef void (*func_ptr)(void); // this is a bad idea

static void null(){} // null function. safer than assigning pointer to NULL

static func_ptr SPI1_next = &null;
static func_ptr SPI2_next = &null;

static uint16_t SPI2_wait_len; // indicates how many bytes we're waiting to receive;

static byte SPI2_RXBF[32]; // buffer received bytes go in while we wait

// values from the data sheet
const static uint8_t ppre_vals[4] = {64,16,4,1}; // primary pre-scale
const static uint8_t spre_vals[8] = {8,7,6,5,4,3,2,1}; // secondary pre-scale

static void getPrescale(uint16_t clock, uint8_t* ppre, uint8_t* spre) {
    int i, j;
    uint16_t clock_khz = CLOCK_FREQUENCY / 1000;
    uint16_t highest_clock = 0;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 8; j++) {
            uint16_t tmp_clock = clock_khz / (ppre_vals[i] * spre_vals[j]);
            if (tmp_clock > highest_clock && tmp_clock <= clock) {
                highest_clock = tmp_clock;
                *ppre = ppre_vals[i];
                *spre = spre_vals[j];
            }
        }
    }
}

static void buffRX() {
    if (SPI2STATbits.SPIRBF) { // if there's data in the RX buffer
        if (SPI2_wait_len > 0) { // if we're expecting data
            SPI2_RXBF[--SPI2_wait_len] = SPI2BUF;
        }
    }
}

void initSPI(uint8_t interface, uint16_t clock, spiMode mode, spiWidth width, spiType mss){

    if (interface == 1) { // Usually DMA
        SPI1STATbits.SPIEN = 0; // Disable for configuration
                
        IFS0bits.SPI1IF = 0; //Clear interrupt flag
        IEC0bits.SPI1IE = 0; //Disable interrupt (so it doesnt mess with this initialization)


        SPI1STATbits.SPISIDL = 0; // Continue module operation in idle mode
        SPI1STATbits.SPIROV = 0; // Clear receive overflow
        SPI1CON1bits.DISSCK = 0; //SPI clock controlled by this module
        SPI1CON1bits.DISSDO = 0; //Output pins are controlled by this module

        SPI1CON1bits.MODE16 = width; //16/8 bit communication mode (1/0)
        SPI1CON2bits.FRMEN = 0; // Framed mode disabled

        if (mss == SPI_MASTER) {
            SPI1CON1bits.MSTEN = 1; // Master mode
            SPI1CON1bits.SMP = 1; // sample input at end of wave
            SPI1CON1bits.SSEN = 0; // clear SS mode    
            TRISBbits.TRISB2 = 0; // set SS pin as output
            
            // clock pre-scale bits
            uint8_t ppre, spre;
            getPrescale(clock, &ppre, &spre);
            SPI1CON1bits.PPRE = ppre;
            SPI1CON1bits.SPRE = spre;
        } else if (mss == SPI_SLAVE) {
            SPI1CON1bits.MSTEN = 0; // Slave mode
            SPI1CON1bits.SMP = 0; // sample input in middle of wave (for slaves, this must always be 0)
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

        IPC2bits.SPI1IP = 4; // Set interrupt priority
        IFS0bits.SPI1IF = 0; //Clear interrupt flag
        IEC0bits.SPI1IE = 1; //Enable interrupt

        SPI1STATbits.SPIEN = 1; //Enable SPI

    } else if (interface == 2) { // Usually IMU / GPS
        SPI2STATbits.SPIEN = 0; // Disable for configuration
        
        IFS2bits.SPI2IF = 0; //Clear interrupt flag
        IEC2bits.SPI2IE = 0; //Disable interrupt (so it doesn't mess with initialization)
        
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
            PORTGbits.RG9 = 0;
            // clock pre-scale bits
            uint8_t ppre, spre;
            getPrescale(clock, &ppre, &spre);
            SPI2CON1bits.PPRE = ppre;
            SPI2CON1bits.SPRE = spre;
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
        
        IPC8bits.SPI2IP = 4; // Set interrupt priority
        IFS2bits.SPI2IF = 0; //Clear interrupt flag
        IEC2bits.SPI2IE = 1; //Enable interrupt

        SPI2STATbits.SPIEN = 1; //Enable SPI

    }
}

void SPI_SS(uint8_t interface, pinState state) {
    if (interface == 1) {
        PORTBbits.RB2 = state;
    } else if (interface == 2) {
        PORTGbits.RG9 = state;
    }
}

byte SPI_TX_RX(uint8_t interface, byte data) {
    if (interface == 1) {
        SPI1BUF = data;
        while (SPI1STATbits.SPITBF){}
        while (!SPI1STATbits.SPIRBF){}
        return SPI1BUF;
    } else if (interface == 2) {
        SPI2BUF = data;
        while (SPI2STATbits.SPITBF){}
        while (!SPI2STATbits.SPIRBF){}
        return SPI2BUF;
    }
    return -1;
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void) {
    IFS0bits.SPI1IF = 0; //Clear interrupt flag
    IFS0bits.SPI1EIF = 0;
    SPI1STATbits.SPIROV = 0;
    (*SPI1_next)(); // next SPI1 operation
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI2Interrupt(void) {
    IFS2bits.SPI2IF = 0; //Clear interrupt flag
    IFS2bits.SPI2EIF = 0;
    SPI2STATbits.SPIROV = 0;
    (*SPI2_next)(); // next SPI2 operation
}