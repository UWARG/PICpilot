/**
 * @file SPI.h
 * @author Serge Babayan
 * @date February 14, 2017, 5:08AM
 * 
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#ifndef SPI_H
#define	SPI_H

/**
 * To avoid synchronization issues, we require the SPI master to send us a first
 * byte that contains this synchronization byte, after which the device should
 * be immediately prepared for transmitting the gps data
 */
#define SPI_SYNCHRONIZATION_BYTE 0x7E

typedef enum {
    SPI_MODE0 = 0,
    SPI_MODE1,
    SPI_MODE2,
    SPI_MODE3
} SPIMode;

/**
 * Sets up the module in SPI slave mode given the mode
 * @param mode
 */
void initSPI(SPIMode mode);

/**
 * Sets up the next SPI transmission by copying over the global gps_data struct
 * over to the spi buffer and calculating its corresponding checksum
 */
void queueTransmitData(void);

#endif

