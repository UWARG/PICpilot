/*
 * File:   InterchipDMA.c
 * Author: Chris Hajduk
 *
 * Created on February 2, 2014, 2:10 PM
 */
#include "InterchipDMA.h"

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
PMData pmData;

char transmitInitialized = 0; //0 = Nothing Received, 1 = Transmit Initialized

unsigned char RxBufferA[sizeof(PMData) + 1] __attribute__((space(dma)));
unsigned char RxBufferB[sizeof(PMData) + 1] __attribute__((space(dma)));
unsigned char TxBufferA[sizeof(PMData) + 1] __attribute__((space(dma)));
unsigned char TxBufferB[sizeof(PMData) + 1] __attribute__((space(dma)));

static unsigned int TxBufferCount = 0; // Keep record of which buffer contains TX Data
static unsigned int RxBufferCount = 0; // Keep record of which buffer contains RX Data

/*
 *
 */


void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void){
    if(RxBufferCount == 0){
        ProcessRxData(&RxBufferA[0]); // Process received SPI data in DMA RAM Primary buffer
    }else{
        ProcessRxData(&RxBufferB[0]); // Process received SPI data in DMA RAM Secondary buffer
    }
    RxBufferCount ^= 1;
    IFS0bits.DMA0IF = 0;// Clear the DMA0 Interrupt Flag
}
void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void){
    if(TxBufferCount == 0){
        ProcessTxData(&TxBufferA[0]); // Copy SPI data into DMA RAM Primary buffer
    }else{
        ProcessTxData(&TxBufferB[0]); // Copy SPI data into DMA RAM Secondary buffer
    }
    TxBufferCount ^= 1;
    IFS0bits.DMA1IF = 0;// Clear the DMA0 Interrupt Flag
}

void ProcessRxData(unsigned char *buffer){
    int i = 0;
    unsigned char *pmDataArray = &pmData;
    char spiChecksum = 0;
//    for (i = 0; i < sizeof(PMData); i++){
//        spiChecksum ^= buffer[i];
//    }
//
//    if (spiChecksum == buffer[sizeof(PMData)]){
        for (i = 0; i < sizeof(PMData); i++){
            pmDataArray[i] = buffer[i];
//            UART1_SendChar(buffer[i]);
        }
    char str[16];
    sprintf(&str,"%f",pmData.time);
    UART1_SendString(str);
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
    DMA0CNT = sizeof(PMData); //+1 for checksum //DMA Transfer Count Length
    DMA0REQ = 0x000A;//0b0100001; //IRQ code for SPI1
    DMA0CONbits.CHEN = 1; //Enable the channel
}

/*SPI TRANSMIT OPERATION*/
void ProcessTxData(unsigned char *buffer){
    int i = 0;
    char *pmDataArray = &pmData;
    char spiChecksum = 0;
    for (i = 0; i < sizeof(PMData); i++){
//        buffer[i] = pmDataArray[i];
        spiChecksum ^= pmDataArray[i];
    }
//    buffer[sizeof(PMData)] = spiChecksum;


}

void init_DMA1(){
    IFS0bits.DMA1IF = 0;
    IEC0bits.DMA1IE = 1;
    DMACS1 = 0; //Clear any IO error flags

    DMA1CONbits.DIR = 1; //Transfer from DSPRAM to SPI
    DMA1CONbits.MODE = 0b10; //Transfer continuously, ping ponging between buffers
    DMA1CONbits.SIZE = 1; //Transfer bytes (8 bits)
    DMA1STA = __builtin_dmaoffset(TxBufferA); //Primary Transfer Buffer
    DMA1STB = __builtin_dmaoffset(TxBufferB); //Secondary Transfer Buffer
    DMA1PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA1CNT = sizeof(PMData); //+1 for checksum //DMA Transfer Count Length
    DMA1REQ = 0x000A;//0b0100001; //IRQ code for SPI1
    DMA1CONbits.CHEN = 1; //Enable the channel
}
#endif

#if PATH_MANAGER
//This is compiled to interface with the GPS chip
GPSData gpsData;

char spiChecksum = 0;
char GPSDataFlag = 0;

unsigned char GPSRxBufferA[sizeof(GPSData) + 1] __attribute__((space(dma)));
unsigned char GPSRxBufferB[sizeof(GPSData) + 1] __attribute__((space(dma)));

static unsigned int GPSRxBufferCount = 0; // Keep record of which buffer contains RX Data
/*
 *
 */

void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void){
    if(GPSRxBufferCount == 0){
        ProcessGPSRxData(&GPSRxBufferA[0]); // Process received SPI data in DMA RAM Primary buffer
    }else{
        ProcessGPSRxData(&GPSRxBufferB[0]); // Process received SPI data in DMA RAM Secondary buffer
    }
    GPSRxBufferCount ^= 1;
    IFS0bits.DMA0IF = 0;// Clear the DMA0 Interrupt Flag
}

void ProcessGPSRxData(unsigned char *buffer){
    int i = 0;
    char *gpsDataArray = &gpsData;
    spiChecksum = 0;
    for (i = 0; i < sizeof(GPSData); i++){
        spiChecksum ^= buffer[i];
    }

    if (spiChecksum == buffer[sizeof(GPSData)]){
        for (i = 0; i < sizeof(GPSData); i++){
            gpsDataArray[i] = buffer[i];
        }
    }
    else{
//        UART1_SendChar('X');
    }

}

//Channel 0 (Highest Priority) must manage receiving data from the GPS
void init_DMA0(){
    IFS0bits.DMA0IF = 0;
    IEC0bits.DMA0IE = 1;
    DMACS0 = 0; //Clear any IO error flags

    DMA0CONbits.DIR = 0; //Transfer from SPI to DSPRAM
    DMA0CONbits.MODE = 0b10; //Transfer continuously, ping ponging between buffers
    DMA0CONbits.SIZE = 1; //Transfer bytes (8 bits)
    DMA0STA = __builtin_dmaoffset(GPSRxBufferA); //Primary Transfer Buffer
    DMA0STB = __builtin_dmaoffset(GPSRxBufferB); //Secondary Transfer Buffer
    DMA0PAD = (volatile unsigned int) &SPI2BUF; //Peripheral Address
    DMA0CNT = sizeof(GPSData); //+1 for checksum //DMA Transfer Count Length
    DMA0REQ = 0b0100001; //IRQ code for SPI2
    DMA0CONbits.CHEN = 1; //Enable the channel
}
#if !ATTITUDE_MANAGER
//TODO: THIS SECTION and add it to the .h file too
 PMData pmData;
//char spiChecksum = 0;
char transmitInitialized = 0;

unsigned char PMRxBufferA[sizeof(PMData) + 1] __attribute__((space(dma)));
unsigned char PMRxBufferB[sizeof(PMData) + 1] __attribute__((space(dma)));
unsigned char PMTxBufferA[sizeof(PMData) + 1] __attribute__((space(dma)));
unsigned char PMTxBufferB[sizeof(PMData) + 1] __attribute__((space(dma)));

static unsigned int PMRxBufferCount = 0; // Keep record of which buffer contains RX Data
static unsigned int PMTxBufferCount = 0; // Keep record of which buffer contains TX Data

void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void){
    if(PMRxBufferCount == 0){
        ProcessPMRxData(&PMRxBufferA[0]); // Process received SPI data in DMA RAM Primary buffer
    }else{
        ProcessPMRxData(&PMRxBufferB[0]); // Process received SPI data in DMA RAM Secondary buffer
    }
    PMRxBufferCount ^= 1;
    IFS0bits.DMA1IF = 0;// Clear the DMA0 Interrupt Flag
}
void __attribute__((__interrupt__, no_auto_psv)) _DMA2Interrupt(void){
    if(PMTxBufferCount == 0){
        ProcessPMTxData(&PMTxBufferA[0]); // Process to place SPI data in DMA RAM Primary buffer
    }else{
        ProcessPMTxData(&PMTxBufferB[0]); // Process to place SPI data in DMA RAM Secondary buffer
    }
    PMTxBufferCount ^= 1;
    IFS1bits.DMA2IF = 0;// Clear the DMA0 Interrupt Flag
}

void ProcessPMTxData(unsigned char *buffer){
    int i = 0;
    char *pmDataArray = &pmData;
    spiChecksum = 0;
    for (i = 0; i < sizeof(PMData); i++){
        buffer[i] = pmDataArray[i];
        spiChecksum ^= pmDataArray[i];
    }
                char str[16];
    sprintf(&str,"%f",pmData.time);
    UART1_SendString(&str);
    buffer[sizeof(PMData)] = spiChecksum;
}

void ProcessPMRxData(unsigned char *buffer){

//    if (!transmitInitialized){
//        DMA2REQbits.FORCE = 1;
//        while (DMA2REQbits.FORCE == 1);
//        transmitInitialized = 1;
//    }

     int i = 0;
    char *pmDataArray = &pmData;
    spiChecksum = 0;
    for (i = 0; i < sizeof(PMData); i++){
        spiChecksum ^= buffer[i];
    }
    if (spiChecksum == buffer[sizeof(PMData)]){
        for (i = 0; i < sizeof(PMData); i++){
//            pmDataArray[i] = buffer[i];
        }
    }
}


void init_DMA1(){
    IFS0bits.DMA1IF = 0;
    IEC0bits.DMA1IE = 1;
    DMACS0 = 0; //Clear any IO error flags

    DMA1CONbits.DIR = 0; //Transfer from SPI to DSPRAM
    DMA1CONbits.MODE = 0b10; //Transfer continuously, ping ponging between buffers
    DMA1CONbits.SIZE = 1; //Transfer bytes (8 bits)
    DMA1STA = __builtin_dmaoffset(PMRxBufferA); //Primary Transfer Buffer
    DMA1STB = __builtin_dmaoffset(PMRxBufferB); //Secondary Transfer Buffer
    DMA1PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA1CNT = sizeof(PMData); //+1 for checksum //DMA Transfer Count Length
    DMA1REQ = 0x000A; //IRQ code for SPI1
    DMA1CONbits.CHEN = 1; //Enable the channel
}
void init_DMA2(){
    IFS1bits.DMA2IF = 0;
    IEC1bits.DMA2IE = 1;
    DMACS0 = 0; //Clear any IO error flags

    DMA2CONbits.DIR = 1; //Transfer from DSPRAM to SPI
    DMA2CONbits.MODE = 0b10; //Transfer continuously, ping ponging between buffers
    DMA2CONbits.SIZE = 1; //Transfer bytes (8 bits)
    DMA2STA = __builtin_dmaoffset(PMTxBufferA); //Primary Transfer Buffer
    DMA2STB = __builtin_dmaoffset(PMTxBufferB); //Secondary Transfer Buffer
    DMA2PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA2CNT = sizeof(PMData); //+1 for checksum //DMA Transfer Count Length
    DMA2REQ = 0x000A; //IRQ code for SPI1
    DMA2CONbits.CHEN = 1; //Enable the channel
}
#endif

#endif



//TODO: Should this really be here? Maybe a separate file for SPI1 and SPI2(VN100)?
#if !PATH_MANAGER
void init_SPI1(){
    //Set interrupts
    IFS0bits.SPI1IF = 0; //Clear interrupt flag
    IEC0bits.SPI1IE = 0; //Disable interrupt (so it doesnt mess with this initialization)

    SPI1BUF = 0;
    //Continue module operation in idle mode
    SPI1STATbits.SPISIDL = 1;
    //SPI clock controlled by this module
    SPI1CON1bits.DISSCK = 0;
    //Output pins are controlled by this module
    SPI1CON1bits.DISSDO = 0;
    //16/8 bit communication mode (1/0)
    SPI1CON1bits.MODE16 = 1; //16
    //Master mode(1)/Slave mode(0)
    SPI1CON1bits.MSTEN = 1; //Master
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
    IEC0bits.SPI1IE = 0; //Disable interrupt

    //Enable SPI
    SPI1STATbits.SPIEN = 1;

    //Then write to the SPI1BUF
    DMA1REQbits.FORCE = 1;
    while (DMA1REQbits.FORCE == 1);
}
#endif


#if PATH_MANAGER
void init_SPI1(){
    //Set interrupts
    IFS0bits.SPI1IF = 0; //Clear interrupt flag
    IEC0bits.SPI1IE = 0; //Disable interrupt (so it doesnt mess with this initialization)

    SPI1BUF = 0;
    //Continue module operation in idle mode
    SPI1STATbits.SPISIDL = 1;
    //SPI clock controlled by this module
    SPI1CON1bits.DISSCK = 0;
    //Output pins are controlled by this module
    SPI1CON1bits.DISSDO = 0;
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

    //Clear Receive Overflow
    SPI1STATbits.SPIROV = 0;

    //Enable SPI
    SPI1STATbits.SPIEN = 1;

    //Then write to the SPI1BUF
    DMA2REQbits.FORCE = 1;
    while (DMA2REQbits.FORCE == 1);

}

void init_SPI2(){
    //Set interrupts
    IFS2bits.SPI2IF = 0; //Clear interrupt flag
    IEC2bits.SPI2IE = 0; //Disable interrupt (so it doesnt mess with this initialization)

    SPI2BUF = 0;
    //Continue module operation in idle mode
    SPI2STATbits.SPISIDL = 1;
    //SPI clock controlled by this module
    SPI2CON1bits.DISSCK = 0;
    //Output pins are not controlled by this module (recieve data only)
    SPI2CON1bits.DISSDO = 1;
    //16/8 bit communication mode (1/0)
    SPI2CON1bits.MODE16 = 1; //16
    //Master mode(1)/Slave mode(0)
    SPI2CON1bits.MSTEN = 0; //Slave
    //Sample Phase (end/middle)
    SPI2CON1bits.SMP = 0; //Sample the input at the end of the square wave
    //Clock Edge Select
    SPI2CON1bits.CKE = 0; //Output data changes from idle state to active clock state (1 is the opposite)
    //Clock Polarity
    SPI2CON1bits.CKP = 0; //Idle clock state is low, active clock state is high
    //Secondary Prescale (The prescale of the prescale)(3 bits)
    SPI2CON1bits.SPRE = 0; //8:1 prescale
    //Primary Prescale (The prescale of the clock) (2 bits)
    SPI2CON1bits.PPRE = 0; //64:1 prescale

    //Then enable interrupts
    IFS2bits.SPI2IF = 0; //Clear interrupt flag
    IEC2bits.SPI2IE = 0; //Disable interrupt

    //Enable SPI
    SPI2STATbits.SPIEN = 1;

    //Then write to the SPI1BUF

}
#endif