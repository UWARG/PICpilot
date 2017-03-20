/**
 * @file SPI.h
 * @author Ian Frosst
 * @date March 13, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef SPI_H
#define	SPI_H

#include "../Common.h"

/*
 * Use this file as a modular way to interface with a device over SPI.
 * 
 * SPI uses 4 pins:
 * SDIx: Serial Data Input              --> 1 : RF7, 2 : RG7
 * SDOx: Serial Data Output             --> 1 : RF8, 2 : RG8
 * SCKx: Shift Clock Input/Output       --> 1 : RF6, 2 : RG6
 * SSx/FSYNCx: Slave select/Frame Sync  --> 1 : RB2, 2 : RG9
 * Slave select is active-low
 * 
 * 
 * Every TX byte/word we send, we'll expect corresponding RX data, of the same size.
 * Whether we care about the received data is context-dependent.
 * If we've just sent our first command, the RX bytes will be useless.
 */


typedef enum {
    PIN_LOW = 0,
    PIN_HIGH
} pin_state;

typedef enum {
    SPI_SLAVE = 0,
    SPI_MASTER
} spi_type;

typedef enum {
    SPI_BYTE = 0,
    SPI_WORD
} spi_width;

typedef enum {
    SPI_MODE0 = 0,
    SPI_MODE1,
    SPI_MODE2,
    SPI_MODE3
} spi_mode;

/**
 * Initializes an SPI port as a master or a slave
 * @param interface Which interface to initialize (1 or 2)
 * @param clock Highest acceptable clock speed (in Hz), if master. Will be set at or below this.
 * @param mode SPI mode 0, 1, 2, or 3. Defines clock polarity and edge.
 * @param width Byte-width or word-width
 * @param master Whether this port will be a master or a slave
 */
void initSPI(uint8_t interface, uint16_t clock, spi_mode mode, spi_width width, spi_type master);

/**
 * Sets the Slave Select pin to the desired state. Not available in slave mode.
 * @param interface SPI interface to set SS for
 * @param state Pin state, high or low
 */
void SPI_SS(uint8_t interface, pin_state state);

/**
 * 
 * @param interface Which interface to send/receive on
 * @param data The data to send
 * @return The data returned
 */
byte SPI_TX_RX(uint8_t interface, byte data);

#endif	/* SPI_H */

