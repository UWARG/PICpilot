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


void __attribute__((__interrupt__,no_auto_psv)) _SPI1Interrupt(void){
    SPI1STATbits.SPIROV = 0;
    IFS0bits.SPI1IF = 0;
    IFS0bits.SPI1EIF = 0;
}


/*SPI RECEIVE OPERATION*/
char transmitInitialized = 0; //0 = Nothing Received, 1 = Transmit Initialized
char newDataAvailable = 0;
char newGPSDataAvailable = 0;

AMData amData __attribute__((space(dma)));
PMData pmData __attribute__((space(dma)));

/*
 *
 */

void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void){
#if !PATH_MANAGER
    if (!transmitInitialized){
        transmitInitialized = 1;
                DMA1REQbits.FORCE = 1;
    while (DMA1REQbits.FORCE == 1);
    }   
#endif
    newDataAvailable = 1;
    IFS0bits.DMA0IF = 0;// Clear the DMA0 Interrupt Flag
}
void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void){
    IFS0bits.DMA1IF = 0;// Clear the DMA0 Interrupt Flag
}

void init_DMA0(){
    IFS0bits.DMA0IF = 0;
    IEC0bits.DMA0IE = 1;
    IPC1bits.DMA0IP = 7; //Highest Priority
    DMACS0 = 0; //Clear any IO error flags

    DMA0CONbits.DIR = 0; //Transfer from SPI to DSPRAM
    DMA0CONbits.AMODE = 0b00; //With post increment mode
    DMA0CONbits.MODE = 0b00; //Transfer continuously
    DMA0CONbits.SIZE = 0; //Transfer words (16 bits)
    DMA0STA = __builtin_dmaoffset(PATH_MANAGER?&amData:&pmData); //Primary Transfer Buffer
    DMA0PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA0CNT = PATH_MANAGER?(sizeof(AMData)/2 + sizeof(AMData) % 2 - 1):(sizeof(PMData)/2 + sizeof(PMData) % 2 - 1); //+1 for checksum //DMA Transfer Count Length
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
    DMA1CONbits.SIZE = 0; //Transfer words (16 bits)
    DMA1STA = __builtin_dmaoffset(PATH_MANAGER?&pmData:&amData); //Primary Transfer Buffer
    DMA1PAD = (volatile unsigned int) &SPI1BUF; //Peripheral Address
    DMA1CNT = PATH_MANAGER?(sizeof(PMData)/2 + sizeof(PMData) % 2 - 1):(sizeof(AMData)/2 + sizeof(AMData) % 2 - 1); //+1 for checksum //DMA Transfer Count Length
    DMA1REQ = 0x000A;//0b0100001; //IRQ code for SPI1
    DMA1CONbits.CHEN = 1; //Enable the channel

}

//TODO: Should this really be here? Maybe a separate file for SPI1 and SPI2(VN100)?
#if !PATH_MANAGER
void init_SPI1(){
    //Set interrupts
    IFS0bits.SPI1IF = 0;
    IEC0bits.SPI1IE = 1;
    IPC2bits.SPI1IP = 4;

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
    //Enable Slave Select
    SPI1CON1bits.SSEN = 0;
    //Sample Phase (end/middle)
    SPI1CON1bits.SMP = 0; //Sample the input at the middle of the square wave
    //Clock Edge Select
    SPI1CON1bits.CKE = 0; //Output data changes from idle state to active clock state (1 is the opposite)
    //Clock Polarity
    SPI1CON1bits.CKP = 0; //Idle clock state is low, active clock state is high
    //Enable SPI
    SPI1STATbits.SPIEN = 1;

    //Then write to the SPI1BUF
}
#endif


#if PATH_MANAGER
void init_SPI1(){
    //Set interrupts
    IFS0bits.SPI1IF = 0;
    IEC0bits.SPI1IE = 1;
    IPC2bits.SPI1IP = 4;

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
    SPI1CON1bits.SPRE = 0b010; //8:1 prescale
    //Primary Prescale (The prescale of the clock) (2 bits)
    SPI1CON1bits.PPRE = 0b00; //64:1 prescale

    //Clear Receive Overflow
    SPI1STATbits.SPIROV = 0;

    //Enable SPI
    SPI1STATbits.SPIEN = 1;

    //Then write to the SPI1BUF


}
#endif

#if PATH_MANAGER
    void init_SPI2(){
    //Set interrupts
    IFS2bits.SPI2IF = 0;
    IEC2bits.SPI2IE = 0;

    SPI2BUF = 0;
    //Continue module operation in idle mode
    SPI2STATbits.SPISIDL = 1;
    //SPI clock controlled by this module
    SPI2CON1bits.DISSCK = 0;
    //Output pins are controlled by this module
    SPI2CON1bits.DISSDO = 0;
    //16/8 bit communication mode (1/0)
    SPI2CON1bits.MODE16 = 1; //16
    //Master mode(1)/Slave mode(0)
    SPI2CON1bits.MSTEN = 0; //Slave
    //Enable Slave Select
    SPI2CON1bits.SSEN = 0;
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

    //Clear Receive Overflow
    SPI2STATbits.SPIROV = 0;

    //Enable SPI
    SPI2STATbits.SPIEN = 1;

    //Then write to the SPI1BUF


}
    char spiChecksum = 0;
char GPSDataFlag = 0;

GPSData gpsData __attribute__((space(dma)));
/*
 *
 */

void __attribute__((__interrupt__, no_auto_psv)) _DMA2Interrupt(void){
    newGPSDataAvailable = 1;
    IFS1bits.DMA2IF = 0;// Clear the DMA0 Interrupt Flag
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
#endif
