/**
 * @file LED.h
 * @author Serj Babayan
 * @created April 21, 2017, 12:43 AM
 *
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#ifndef LED_H
#define	LED_H

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

