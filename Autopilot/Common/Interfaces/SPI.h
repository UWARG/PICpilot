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
} spi_mode ;

void initSPI(uint8_t interface, uint16_t clock, spi_mode mode);

#endif	/* SPI_H */

