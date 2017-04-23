/**
 * @file SPI.c
 * @author Serj Babayan
 * @date April 23, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#include "SPI.h"
#include "GPS.h"
#include <xc.h>

static uint8_t spi_buffer[sizeof(GPSData) + 1];
static uint8_t spi_buffer_size = sizeof(GPSData) + 1;

static uint16_t spi_buffer_index = 0; //index of the packet we're currently transmitting

// indicates if we're currently transmitting the gps data, or waiting for the synchronization byte
static bool currently_transmitting = false;

void initSPI(SPIMode mode){
    SPI1STATbits.SPIEN = 0; // Disable for configuration

    IFS0bits.SPI1IF = 0; //Clear interrupt flag
    IEC0bits.SPI1IE = 0; //Disable interrupt (so it doesn't mess with initialization)
    
    SPI1BUF = 0; //clear the buffer
    
    SPI1STATbits.SPISIDL = 0; //Continue module operation in idle mode
    SPI1STATbits.SPIROV = 0; // Clear receive overflow
    SPI1CON1bits.DISSCK = 0; //SPI clock controlled by this module (internal clock)
    SPI1CON1bits.DISSDO = 0; //Output pins are controlled by this module (SDOx pin)
    SPI1CON1bits.MODE16 = 1; // Communication is byte-wide (8 bits)
    SPI1CON1bits.SMP = 0; // Input data is sampled at the middle of data
    
    SPI1CON2bits.FRMEN = 0; // disable framed mode

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
   
    SPI1CON1bits.MSTEN = 0; // Slave mode
    SPI1CON1bits.SMP = 0; // sample input in middle of wave
    SPI1CON1bits.SSEN = 0; // disable SS?

    IPC2bits.SPI1IP = 4; // Set interrupt priority
    IFS0bits.SPI1IF = 0; //Clear interrupt flag
    IEC0bits.SPI1IE = 1; //Enable interrupt
    SPI1STATbits.SPIEN = 1; //Enable SPI
}

void queueTransmitData(void){
    uint16_t i = 0;
    uint8_t checksum = 0;
    for (i = 0; i < sizeof(GPSData); i++){
        spi_buffer[i] = ((uint8_t*)(&gps_data))[i];
        checksum += ((uint8_t*)(&gps_data))[i] + i;
    }

    spi_buffer[i] = checksum; //add the checksum to the end of the transmission=
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void) {
    if (currently_transmitting){
        SPI1BUF = spi_buffer[spi_buffer_index];
        if (spi_buffer_index < spi_buffer_size){
            spi_buffer_index++;
        } else {
            currently_transmitting = false;
        }
    } else if (SPI1BUF == SPI_SYNCHRONIZATION_BYTE){
        SPI1BUF = spi_buffer[0];
        currently_transmitting = true;
        spi_buffer_index = 1; //start the transmission. Subsequent
    }
    
    IFS0bits.SPI1IF = 0; // Clear the Interrupt Flag
}