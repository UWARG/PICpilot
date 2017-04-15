/*
 * File:   InterchipDMA.c
 * Author: Chris Hajduk
 *
 * Created on February 2, 2014, 2:10 PM
 */
#include "InterchipDMA.h"
#include "../Common.h"
#include <stdbool.h>


/*SPI RECEIVE OPERATION*/
volatile bool DMADataAvailable = 0;

AMData amData __attribute__((space(dma)));
PMData pmData __attribute__((space(dma)));

/*
 * DMA0 Interrupt 
 */
void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void){
    DMADataAvailable = 1;
    IFS0bits.DMA0IF = 0; //clear the interrupt flag
}
        
void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void){
    IFS0bits.DMA1IF = 0; //clear the interrupt flag
}

char isDMADataAvailable(){
    return DMADataAvailable;
}

// receiving data
void init_DMA0(char isAttMan){
    IFS0bits.DMA0IF = 0;
    IEC0bits.DMA0IE = 1;
    IPC1bits.DMA0IP = 7; //Highest Priority
    DMACS0 = 0; //Clear any IO error flags

    DMA0CONbits.DIR = 0; //Transfer from SPI to DSPRAM
    DMA0CONbits.AMODE = 0b00; //With post increment mode
    DMA0CONbits.MODE = 0b00; //Transfer continuously. Ping pong mode disabled
    DMA0CONbits.SIZE = 1; //Transfer byte (8 bits)
    if (isAttMan == 1) {
        DMA0STA = __builtin_dmaoffset(&pmData); //Primary Transfer Buffer
        DMA0CNT = (sizeof(PMData) - 1); //+1 for checksum //DMA Transfer Count Length
    } else {
        DMA0STA = __builtin_dmaoffset(&amData); //Primary Transfer Buffer
        DMA0CNT = (sizeof(AMData) - 1); //+1 for checksum //DMA Transfer Count Length   
    }
    DMA0PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA0REQ = 0x000A;//0b0100001; //IRQ code for SPI1
    DMA0CONbits.CHEN = 1; //Enable the channel
}

// sending data
void init_DMA1(char isAttMan){
    IFS0bits.DMA1IF = 0;
    IEC0bits.DMA1IE = 1;
    IPC3bits.DMA1IP = 7;
    DMACS1 = 0; //Clear any IO error flags

    DMA1CONbits.DIR = 1; //Transfer from DSPRAM to SPI
    DMA1CONbits.AMODE = 0b00; //With post increment mode
    DMA1CONbits.MODE = 0b00; //Transfer continuously, ping ponging between buffers
    DMA1CONbits.SIZE = 1; //Transfer byte (8 bits)
    if (isAttMan == 1) {
        DMA1STA = __builtin_dmaoffset(&amData); //Primary Transfer Buffer
        DMA1CNT = (sizeof(AMData) - 1); //+1 for checksum //DMA Transfer Count Length
    } else {
        DMA1STA = __builtin_dmaoffset(&pmData); //Primary Transfer Buffer
        DMA1CNT = (sizeof(PMData) - 1); //+1 for checksum //DMA Transfer Count Length   
    }
    DMA1PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA1REQ = 0x000A;//0b0100001; //IRQ code for SPI1
    DMA1CONbits.CHEN = 1; //Enable the channel
}
