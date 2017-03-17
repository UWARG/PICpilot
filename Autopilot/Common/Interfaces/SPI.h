/**
 * @file SPI.h
 * @author Ian Frosst
 * @date March 13, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef SPI_H
#define	SPI_H

#include <stdint.h>

typedef uint8_t byte;
typedef uint16_t word;

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
 * Sets the Slave Select pin to the desired state. Not available in slave mode.
 * @param interface SPI interface to set SS for
 * @param state Pin state, high or low
 */
void SPI_SS(uint8_t interface, pin_state state);

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
 * 
 * @param interface Which interface to send/receive on
 * @param data The data to send
 * @return The data returned
 */
byte SPI_TX_RX(uint8_t interface, byte data);

#endif	/* SPI_H */

