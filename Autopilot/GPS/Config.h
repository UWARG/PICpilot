/**
 * @file Config.h
 * @author Serj Babayan
 */

#ifndef CONFIG_H
#define	CONFIG_H

#include <xc.h>

/**
 * What SPI mode to transfer the gps data over
 */
#define SPI_MODE SPI_MODE3

/**
 * Tells the module to bypass the parsing functionality and just output the
 * received strings from the GPS module to the TX buffer
 */
#define DEBUG_TX_GPS 1

#endif
