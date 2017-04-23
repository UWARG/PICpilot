/*
 * @file WARG_GPS.h
 * @author Serj Babayan
 * @created April 23, 2017, 2:07 PM
 *
 * DMA channel 2 is used for receiving data, DMA channel 3 is used for sending data
 */

#include <stdbool.h>
#include "../../Common/Interfaces/SPI.h"
#include "WARG_GPS.h"
#include "../Peripherals/GPS.h"
#include "../../Common/Utilities/Logger.h"
#include <string.h>

GPSData gps_data;

static uint8_t gps_receive_buffer[sizeof(GPSData)];

static bool data_available = false;

static bool connected = false;

static void initDMA2(void);
static void initDMA3(void);

static volatile uint8_t dma2_space[sizeof(GPSData) + 2] __attribute__((space(dma))); //one byte for the checksum, and another for the synchronization byte
static volatile uint8_t dma3_space[sizeof(GPSData) + 2] __attribute__((space(dma)));

/**
 * Initializes communications with the GPS
 */
void initGPS(){
    initSPI(WARG_GPS_SPI_INTERFACE, WARG_GPS_SPI_FREQ_KHZ, WARG_GPS_SPI_MODE, 8, SPI_MASTER);
    initDMA2();
    initDMA3();
}

void requestGPSInfo(){
    SPI_SS(2, 1);
    //trigger a DMA send
    DMA3CONbits.CHEN = 1;
    DMA3REQbits.FORCE = 1;
}

bool isNewGPSDataAvailable(){
    if (data_available){
        data_available = false;
        return true;
    }
    return false;
}

bool isGPSConnected(){
    return connected;
}

// receiving data
static void initDMA2()
{
    DMA2CONbits.CHEN = 0; //disable the channel for now
    IFS1bits.DMA2IF = 0;
    IEC1bits.DMA2IE = 1;
    IPC6bits.DMA2IP = 7; //Highest Priority
    DMACS0 = 0; //Clear any IO error flags

    DMA2CONbits.DIR = 0; //Transfer from SPI to DSPRAM
    DMA2CONbits.AMODE = 0b00; //With post increment mode
    DMA2CONbits.MODE = 0b00; //Continuous transfer mode, ping pong disabled
    DMA2CONbits.SIZE = 1; //Transfer byte (8 bits)
    DMA2CONbits.HALF = 0; //Initiate dma interrupt when all of the data has been moved

    DMA2STA = __builtin_dmaoffset(&dma2_space); //Primary Transfer Buffer
    DMA2CNT = (sizeof(dma2_space) - 1); //count is 0-indexed, so -1

    DMA2PAD = (volatile unsigned int) &SPI2BUF; //Peripheral Address
    DMA2REQ = 0b0100001; ////IRQ code for SPI2
    DMA2CONbits.CHEN = 1; //Enable the channel
}

// sending data

static void initDMA3()
{
    DMA3CONbits.CHEN = 0; //disable the channel for now
    IFS2bits.DMA3IF = 0;
    IEC2bits.DMA3IE = 1;
    IPC9bits.DMA3IP = 7;
    DMACS1 = 0; //Clear any IO error flags

    DMA3CONbits.DIR = 1; //Transfer from DSPRAM to SPI
    DMA3CONbits.AMODE = 0b00; //With post increment mode

    DMA3CONbits.MODE = 0b01; //One shot transfer, ping pong mode disabled

    DMA3CONbits.SIZE = 1; //Transfer byte (8 bits)
    DMA3CONbits.HALF = 0; //Initiate dma interrupt when all of the data has been moved

    DMA3STA = __builtin_dmaoffset(&dma3_space); //Primary Transfer Buffer
    DMA3CNT = (sizeof(dma3_space) - 1); //count is 0-indexed, so -1

    DMA3PAD = (volatile unsigned int) &SPI2BUF; //Peripheral Address
    DMA3REQ = 0b0100001; ////IRQ code for SPI2

    //first byte should always be the synchronization byte
    dma3_space[0] = WARG_GPS_SPI_SYNCHRONIZATION_BYTE;
    
    DMA3CONbits.CHEN = 1; //Enable the channel
}

/*
 * Called when we've just received data from our SPI buffers
 */
void __attribute__((__interrupt__, no_auto_psv)) _DMA2Interrupt(void)
{
    debug("received dma data");
    uint8_t checksum = 0;
    uint16_t i = 0;

    for (i = 0; i < sizeof(GPSData); i++) { //go through all the bytes, not including the checksum or the first synchronization byte
        gps_receive_buffer[i] = dma2_space[i + 1];
        checksum += dma2_space[i + 1] + i;
    }

    if (checksum == dma2_space[i]){
        debug("checksum passed!");
        connected = true;
        data_available = false;
        memcpy((uint8_t*)&gps_data, gps_receive_buffer, sizeof(GPSData));
        data_available = true;
    } else {
        debug("checksum failed!");
        debugArray(dma2_space, sizeof(dma2_space));
    }

    IFS1bits.DMA2IF = 0; //clear the interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA3Interrupt(void)
{
    IFS2bits.DMA3IF = 0; //clear the interrupt flag
}
