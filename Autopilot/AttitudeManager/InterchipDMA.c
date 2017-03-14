/*
 * File:   InterchipDMA.c
 * Author: Chris Hajduk
 *
 * Created on February 2, 2014, 2:10 PM
 */
#include "InterchipDMA.h"
#include "AttitudeManager.h"

void __attribute__((__interrupt__,no_auto_psv)) _SPI1Interrupt(void){
    SPI1STATbits.SPIROV = 0;
    IFS0bits.SPI1IF = 0;
    IFS0bits.SPI1EIF = 0;
}

/*SPI RECEIVE OPERATION*/
char DMADataAvailable = 0;

AMData amData __attribute__((space(dma)));
PMData pmData __attribute__((space(dma)));

/*
 * DMA0 Interrupt (with reset)
 */
void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void){
//    IEC0bits.DMA0IE = 0; // Disable interrupts (we don't want another reset while we're doing this one)
//#if PATH_MANAGER
//    // if received bad checksum
//    if (amData.checkbyteDMA != 0xAB && amData.checkbyteDMA != 0xFFAB) {
//        INTERCOM_4 = 1; // notify AM
//        while(!INTERCOM_2); // wait until AM accepts
//#elif ATTITUDE_MANAGER
//    if (INTERCOM_4) { // if PM requested reset
//        INTERCOM_2 = 1; // notify PM
//#if DEBUG
//        //Variable only used for debugging
//        int counter = 0;
//#endif
//        while(!INTERCOM_4){
//#if DEBUG
//            //Warning if chip is stuck here
//            if (counter++ > 20){
//                warning("Intercom is late. Lag or Software error?1");
//            }
//#endif
//        };
//#endif
//        SPI1STATbits.SPIEN = 0; //Disable SPI1
//        DMA0CONbits.CHEN = 0; //Disable DMA0 channel
//        DMA1CONbits.CHEN = 0; //Disable DMA1 channel
//        while(SPI1STATbits.SPIRBF) { //Clear SPI1
//            int dummy = SPI1BUF;
//#if DEBUG
//            //Warning if chip is stuck here
//            if (counter++ > 20){
//                debug("Intercom is late. Lag or Software error?2");
//            }
//#endif
//        }
//        // Clear flags
//#if PATH_MANAGER
//        INTERCOM_4 = 0;
//        while(INTERCOM_2);
//#elif ATTITUDE_MANAGER
//        INTERCOM_2 = 0;
//        while(INTERCOM_4){            //Warning if chip is stuck here
//            if (counter++ > 20){
//                debug("Intercom is late. Lag or Software error?3");
//            }}
//#endif
//        init_SPI1(); // Restart SPI
//        init_DMA0(); // Restart DMA0
//        init_DMA1(); // Restart DMA1
//        DMA1REQbits.FORCE = 1;
//        while (DMA1REQbits.FORCE == 1){
//#if DEBUG
//            //Warning if chip is stuck here
//            if (counter++ > 20){
//                debug("Intercom is late. Lag or Software error?4");
//            }
//#endif
//        }
//    }
    DMADataAvailable = 1;
    IFS0bits.DMA0IF = 0;// Clear the DMA1 Interrupt Flag
//    IEC0bits.DMA0IE = 1; // Enable DMA0 Interrupts
}
        
void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void){
    IFS0bits.DMA1IF = 0;// Clear the DMA0 Interrupt Flag
}

char isDMADataAvailable(){
    return DMADataAvailable;
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
    DMA0STA = __builtin_dmaoffset(&pmData); //Primary Transfer Buffer
    DMA0PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA0CNT = (sizeof(PMData) - 1); //+1 for checksum //DMA Transfer Count Length
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
    DMA1STA = __builtin_dmaoffset(&amData); //Primary Transfer Buffer

    DMA1PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA1CNT = (sizeof(AMData) - 1); //+1 for checksum //DMA Transfer Count Length
    DMA1REQ = 0x000A;//0b0100001; //IRQ code for SPI1
    DMA1CONbits.CHEN = 1; //Enable the channel

}
