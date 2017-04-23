/*
 * File:   InterchipDMA.c
 * Author: Chris Hajduk
 *
 * Created on February 2, 2014, 2:10 PM
 */
#include "GPSDMA.h"


///*SPI RECEIVE OPERATION*/
//char newGPSDataAvailable = 0;
//
//
//GPSData gpsData __attribute__((space(dma))); //Moved line outside Compiler Statement for a Quick Fix.... Needs to be turned on either wa for both GPS's
///*
// *
// */
//
//void __attribute__((__interrupt__, no_auto_psv)) _DMA2Interrupt(void){
//    newGPSDataAvailable = 1;
//    IFS1bits.DMA2IF = 0;// Clear the DMA2 Interrupt Flag
//}
//
//void init_DMA2(){
//    IFS1bits.DMA2IF = 0;
//    IEC1bits.DMA2IE = 1;
//
//    DMA2CONbits.AMODE = 0b00; //Register Indirect Mode
//    DMA2CONbits.DIR = 0; //Transfer from SPI to DSPRAM
//    DMA2CONbits.MODE = 0b00; //Transfer continuously
//    DMA2CONbits.SIZE = 1; //Transfer bytes (8 bits)
//    DMA2STA = __builtin_dmaoffset(&gpsData); //Primary Transfer Buffer
//    DMA2PAD = (volatile unsigned int) &SPI2BUF; //Peripheral Address
//    DMA2CNT = sizeof(GPSData) - 1; //+1 for checksum //DMA Transfer Count Length
//    DMA2REQ = 0b0100001; //IRQ code for SPI2
//    DMA2CONbits.CHEN = 1; //Enable the channel
//}
