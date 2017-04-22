/**
 * @file Logger.h
 * @author Chris Hajduk, Serj Babayan
 * @created August 24, 2014
 * Provides functions for outputting serial messages via one of the UART lines. 
 * Contains three levels of messages: warnings, errors, and debug
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef LOGGER_H
#define	LOGGER_H

#include <stdint.h>

/**
 * Which UART interface to use for the logger (1 or 2)
 */
#define LOGGER_UART_INTERFACE 2

/**
 * Baud rate for the interface
 */
#define LOGGER_UART_BAUD_RATE 9600

/**
 * How long to make the string buffer. This should be the maximum length of the string
 * that can be logged, in bytes
 */
#define LOGGER_BUFFER_INITIAL_LENGTH 100
#define LOGGER_BUFFER_MAX_LENGTH 100

/**
 * Initializes the logger module by initializing the specified UART channel
 */
void initLogger(void);

/**
 * Output a info level message
 * @param message
 */
void debug(char* message);

#endif
