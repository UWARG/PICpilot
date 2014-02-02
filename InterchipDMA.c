/*
 * File:   InterchipDMA.c
 * Author: Chris Hajduk
 *
 * Created on February 2, 2014, 2:10 PM
 */
#include "InterchipDMA.h"
#include "main.h"

#if ATTITUDE_MANAGER
#include "AttitudeManager.h"
#endif

#if COMMUNICATION_MANAGER
#include "net.h"
#endif

#if PATH_MANAGER
#include "PathManager.h"
#endif


#if !PATH_MANAGER

/*SPI RECEIVE OPERATION*/
struct PMData {
    float time;     //4 Bytes   -  hhmmss.ssss
    long double latitude;  //8 Bytes - ddd.mmmmmm
    long double longitude; //8 Bytes - ddd.mmmmmm
    float altitude; // Meters
    float heading;  //Degrees
    float speed;    //KM/H
    char satellites;    //1 Byte
    char positionFix;   //0 = No GPS, 1 = GPS fix, 2 = DGSP Fix
};
struct PMData pmData;

char transmitInitialized = 0; //0 = Nothing Received, 1 = Transmit Initialized

unsigned char RxBufferA[sizeof(struct PMData) + 1] __attribute__((space(dma)));
unsigned char RxBufferB[sizeof(struct PMData) + 1] __attribute__((space(dma)));
unsigned char TxBufferA[sizeof(struct PMData) + 1] __attribute__((space(dma)));
unsigned char TxBufferB[sizeof(struct PMData) + 1] __attribute__((space(dma)));


/*
 *
 */


void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void){

    if (!transmitInitialized){
        DMA1REQbits.FORCE = 1;
        while (DMA1REQbits.FORCE == 1);
    }

    static unsigned int BufferCount = 0; // Keep record of which buffer contains RX Data
    if(BufferCount == 0){
        ProcessRxData(&RxBufferA[0]); // Process received SPI data in DMA RAM Primary buffer
    }else{
        ProcessRxData(&RxBufferB[0]); // Process received SPI data in DMA RAM Secondary buffer
    }
    BufferCount ^= 1;
    IFS0bits.DMA0IF = 0;// Clear the DMA0 Interrupt Flag
}
void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void){
    static unsigned int BufferCount = 0; // Keep record of which buffer contains TX Data
    if(BufferCount == 0){
        ProcessTxData(&TxBufferA[0]); // Copy SPI data into DMA RAM Primary buffer
    }else{
        ProcessTxData(&TxBufferB[0]); // Copy SPI data into DMA RAM Secondary buffer
    }
    BufferCount ^= 1;
    IFS0bits.DMA1IF = 0;// Clear the DMA0 Interrupt Flag
}

void ProcessRxData(unsigned char *buffer){
    int i = 0;
    unsigned char *pmDataArray = &pmData;
    char spiChecksum = 0;
//    for (i = 0; i < sizeof(struct PMData); i++){
//        spiChecksum ^= buffer[i];
//    }
//
//    if (spiChecksum == buffer[sizeof(struct PMData)]){
        for (i = 0; i < sizeof(struct PMData); i++){
            pmDataArray[i] = buffer[i];
        }
//    }

}

void init_DMA0(){
    IFS0bits.DMA0IF = 0;
    IEC0bits.DMA0IE = 1;
    DMACS0 = 0; //Clear any IO error flags

    DMA0CONbits.DIR = 0; //Transfer from SPI to DSPRAM
    DMA0CONbits.MODE = 0b10; //Transfer continuously, ping ponging between buffers
    DMA0CONbits.SIZE = 1; //Transfer bytes (8 bits)
    DMA0STA = __builtin_dmaoffset(RxBufferA); //Primary Transfer Buffer
    DMA0STB = __builtin_dmaoffset(RxBufferB); //Secondary Transfer Buffer
    DMA0PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA0CNT = sizeof(struct PMData); //+1 for checksum //DMA Transfer Count Length
    DMA0REQ = 0x000A;//0b0100001; //IRQ code for SPI1
    DMA0CONbits.CHEN = 1; //Enable the channel
}

/*SPI TRANSMIT OPERATION*/
void ProcessTxData(unsigned char *buffer){
    int i = 0;
    char *pmDataArray = &pmData;
    char spiChecksum = 0;
    for (i = 0; i < sizeof(struct PMData); i++){
        buffer[i] = pmDataArray[i];
        spiChecksum ^= pmDataArray[i];
    }
    buffer[sizeof(struct PMData)] = spiChecksum;


}

void init_DMA1(){
    IFS0bits.DMA1IF = 0;
    IEC0bits.DMA1IE = 1;
    DMACS1 = 0; //Clear any IO error flags

    DMA1CONbits.DIR = 0; //Transfer from SPI to DSPRAM
    DMA1CONbits.MODE = 0b10; //Transfer continuously, ping ponging between buffers
    DMA1CONbits.SIZE = 1; //Transfer bytes (8 bits)
    DMA1STA = __builtin_dmaoffset(RxBufferA); //Primary Transfer Buffer
    DMA1STB = __builtin_dmaoffset(RxBufferB); //Secondary Transfer Buffer
    DMA1PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA1CNT = sizeof(struct PMData); //+1 for checksum //DMA Transfer Count Length
    DMA1REQ = 0x000A;//0b0100001; //IRQ code for SPI1
    DMA1CONbits.CHEN = 1; //Enable the channel
}
#endif

//TODO: Should this really be here? Maybe a separate file for SPI1 and SPI2(VN100)?
void init_SPI(){
    //Set interrupts
    IFS0bits.SPI1IF = 0; //Clear interrupt flag
    IEC0bits.SPI1IE = 0; //Disable interrupt (so it doesnt mess with this initialization)

    SPI1BUF = 0;
    //Continue module operation in idle mode
    SPI1STATbits.SPISIDL = 1;
    //SPI clock controlled by this module
    SPI1CON1bits.DISSCK = 0;
    //Output pins are not controlled by this module (recieve data only)
    SPI1CON1bits.DISSDO = 1;
    //16/8 bit communication mode (1/0)
    SPI1CON1bits.MODE16 = 1; //16
    //Master mode(1)/Slave mode(0)
    SPI1CON1bits.MSTEN = 0; //Slave
    //Sample Phase (end/middle)
    SPI1CON1bits.SMP = 0; //Sample the input at the end of the square wave
    //Clock Edge Select
    SPI1CON1bits.CKE = 0; //Output data changes from idle state to active clock state (1 is the opposite)
    //Clock Polarity
    SPI1CON1bits.CKP = 0; //Idle clock state is low, active clock state is high
    //Secondary Prescale (The prescale of the prescale)(3 bits)
    SPI1CON1bits.SPRE = 0; //8:1 prescale
    //Primary Prescale (The prescale of the clock) (2 bits)
    SPI1CON1bits.PPRE = 0; //64:1 prescale

    //Then enable interrupts
    IFS0bits.SPI1IF = 0; //Clear interrupt flag
    IEC0bits.SPI1IE = 0; //Enable interrupt

    //Enable SPI
    SPI1STATbits.SPIEN = 1;

    //Then write to the SPI1BUF

}