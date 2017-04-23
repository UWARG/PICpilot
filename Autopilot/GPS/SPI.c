#include <xc.h>

void initSPI(){
    //initialize intercom SPI port
    TRISBbits.TRISB12 = 0; //SPI data out
    TRISBbits.TRISB13 = 0; //SPI clk out
    TRISBbits.TRISB15 = 0; //SPI slave select out
    PORTBbits.RB15 = 1; //slave disabled

    SPI1BUF = 0;
    IFS0bits.SPI1IF = 0; // Clear the Interrupt Flag
    IEC0bits.SPI1IE = 0; // Disable the Interrupt
    //SPI1CON1 Register Settings
    SPI1CON1bits.DISSCK = 0; // Internal Serial Clock is Enabled
    SPI1CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
    SPI1CON1bits.MODE16 = 1; // Communication is byte-wide (8 bits)
    SPI1CON1bits.SMP = 0; // Input data is sampled at the middle of data
    // output time
    SPI1CON1bits.CKE = 0; // Serial output data changes on transition
    // from Idle clock state to active clock state
    SPI1CON1bits.CKP = 0; // Idle state for clock is a low level; active
    // state is a high level
//    SPI1CON1bits.SSEN = 1;	// Enable slave select pin - don't uncomment this UART won't work
    SPI1CON1bits.SPRE = 0b110;
    SPI1CON1bits.PPRE = 0b01;
    SPI1CON1bits.MSTEN = 1; // Master mode Enabled
    SPI1STATbits.SPIROV = 0; // SPI recieve overflow
    SPI1STATbits.SPIEN = 1; // Enable SPI module

    // Interrupt Controller Settings
    IFS0bits.SPI1IF = 0; // Clear the Interrupt Flag
    IEC0bits.SPI1IE = 1; // Enable the Interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void) {
    //if(SPI1BUF == 1) PORTFbits.RF4 = 1;
    //else PORTFbits.RF4 = 0;
    //        PORTAbits.RA6 = 1;

    IFS0bits.SPI1IF = 0; // Clear the Interrupt Flag
}