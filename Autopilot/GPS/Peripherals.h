/**
 * @file Peripherals.h
 * @author Serj Babayan
 * @created April 21, 2017, 12:43 AM
 */

#ifndef PERIPHERALS_H
#define	PERIPHERALS_H

#include  <stdbool.h>

/**
 * Initializes A6 pin as the LED pin
 * @return
 */
void initLED(void);

/**
 * Turn on/off the status LED
 * @return
 */
void setLED(bool on);

#endif

