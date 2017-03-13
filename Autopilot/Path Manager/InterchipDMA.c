/*
 * File:   InterchipDMA.c
 * Author: Chris Hajduk
 *
 * Created on February 2, 2014, 2:10 PM
 */
#include "InterchipDMA.h"

void __attribute__((__interrupt__,no_auto_psv)) _SPI1Interrupt(void){
    SPI1STATbits.SPIROV = 0;
    IFS0bits.SPI1IF = 0;
    IFS0bits.SPI1EIF = 0;
}

/*SPI RECEIVE OPERATION*/
char transmitInitialized = 0; //0 = Nothing Received, 1 = Transmit Initialized
char newDMADataAvailable = 0;
char newGPSDataAvailable = 0;

AMData amData __attribute__((space(dma)));
PMData pmData __attribute__((space(dma)));

/*
 * DMA0 Interrupt (with reset)
 */
void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void){
//    IEC0bits.DMA0IE = 0; // Disable interrupts (we don't want another reset while we're doing this one)
//    // if received bad checksum
//    if (amData.checkbyteDMA != 0xAB && amData.checkbyteDMA != 0xFFAB) {
//        INTERCOM_4 = 1; // notify AM
//        while(!INTERCOM_2); // wait until AM accepts
//        SPI1STATbits.SPIEN = 0; //Disable SPI1
//        DMA0CONbits.CHEN = 0; //Disable DMA0 channel
//        DMA1CONbits.CHEN = 0; //Disable DMA1 channel
//        while(SPI1STATbits.SPIRBF) { //Clear SPI1
//            int dummy = SPI1BUF;
//        }
//        // Clear flags
//        INTERCOM_4 = 0;
//        while(INTERCOM_2);
//        init_SPI1(); // Restart SPI
//        init_DMA0(); // Restart DMA0
//        init_DMA1(); // Restart DMA1
//        DMA1REQbits.FORCE = 1;
//        while (DMA1REQbits.FORCE == 1);
//    }
    newDMADataAvailable = 1;
    IFS0bits.DMA0IF = 0;// Clear the DMA1 Interrupt Flag
//    IEC0bits.DMA0IE = 1; // Enable DMA0 Interrupts
}
        
void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void){
    IFS0bits.DMA1IF = 0;// Clear the DMA0 Interrupt Flag
}

char isDMADataAvailable(){
    return newDMADataAvailable;
}

void init_DMA0(){
    IFS0bits.DMA0IF = 0;
    IEC0bits.DMA0IE = 1;
    IPC1bits.DMA0IP = 7; //Highest Priority
    DMACS0 = 0; //Clear any IO error flags

    DMA0CONbits.DIR = 0; //Transfer from SPI to DSPRAM
    DMA0CONbits.AMODE = 0b00; //With post increment mode
    DMA0CONbits.MODE = 0b00; //Transfer continuously
    DMA0CONbits.SIZE = 1; //Transfer byte (8 bits)
    DMA0STA = __builtin_dmaoffset(&amData); //Primary Transfer Buffer
    DMA0PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA0CNT = (sizeof(AMData) - 1); //+1 for checksum //DMA Transfer Count Length
    DMA0REQ = 0x000A;//0b0100001; //IRQ code for SPI1
    DMA0CONbits.CHEN = 1; //Enable the channel
}

void init_DMA1(){
    IFS0bits.DMA1IF = 0;
    IEC0bits.DMA1IE = 1;
    IPC3bits.DMA1IP = 7;
    DMACS1 = 0; //Clear any IO error flags

    DMA1CONbits.DIR = 1; //Transfer from DSPRAM to SPI
    DMA1CONbits.AMODE = 0b00; //Without post increment mode
    DMA1CONbits.MODE = 0b00; //Transfer continuously, ping ponging between buffers
    DMA1CONbits.SIZE = 1; //Transfer byte (8 bits)
    DMA1STA = __builtin_dmaoffset(&pmData); //Primary Transfer Buffer
    DMA1PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA1CNT = (sizeof(PMData) - 1); //DMA Transfer Count Length
    DMA1REQ = 0x000A;//0b0100001; //IRQ code for SPI1
    DMA1CONbits.CHEN = 1; //Enable the channel

}

char spiChecksum = 0;
char GPSDataFlag = 0;

GPSData gpsData __attribute__((space(dma))); //Moved line outside Compiler Statement for a Quick Fix.... Needs to be turned on either wa for both GPS's
/*
 *
 */
void __attribute__((__interrupt__,no_auto_psv)) _SPI2Interrupt(void){
    SPI2STATbits.SPIROV = 0;
    IFS2bits.SPI2IF = 0;
    IFS2bits.SPI2EIF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA2Interrupt(void){
    newGPSDataAvailable = 1;
    IFS1bits.DMA2IF = 0;// Clear the DMA2 Interrupt Flag
}

void init_DMA2(){
    IFS1bits.DMA2IF = 0;
    IEC1bits.DMA2IE = 1;

    DMA2CONbits.AMODE = 0b00; //Register Indirect Mode
    DMA2CONbits.DIR = 0; //Transfer from SPI to DSPRAM
    DMA2CONbits.MODE = 0b00; //Transfer continuously
    DMA2CONbits.SIZE = 1; //Transfer bytes (8 bits)
    DMA2STA = __builtin_dmaoffset(&gpsData); //Primary Transfer Buffer
    DMA2PAD = (volatile unsigned int) &SPI2BUF; //Peripheral Address
    DMA2CNT = sizeof(GPSData) - 1; //+1 for checksum //DMA Transfer Count Length
    DMA2REQ = 0b0100001; //IRQ code for SPI2
    DMA2CONbits.CHEN = 1; //Enable the channel
}
