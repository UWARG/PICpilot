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

void initSPI(uint8_t interface, uint16_t clock, spi_mode mode, spi_width width, spi_type mss);

#endif	/* SPI_H */

