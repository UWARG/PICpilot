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
#define LOGGER_UART_INTERFACE 1

/**
 * Baud rate for the interface
 */
#define LOGGER_UART_BAUD_RATE 115200

/**
 * How long to make the string buffer. This should be the maximum length of the string
 * that can be logged, in bytes
 */
#define LOGGER_BUFFER_INITIAL_LENGTH 100
#define LOGGER_BUFFER_MAX_LENGTH 1000

/**
 * Label prefixes to inject for each type of message
 */
#define DEBUG_TAG_STRING "[DEBUG]"
#define DEBUG_TAG_STRING_LENGTH 7
#define ERROR_TAG_STRING "[ERROR]"
#define ERROR_TAG_STRING_LENGTH 7
#define WARNING_TAG_STRING "[WARNING]"
#define WARNING_TAG_STRING_LENGTH 9
#define INFO_TAG_STRING "[INFO]"
#define INFO_TAG_STRING_LENGTH 6

/**
 * Initializes the logger module by initializing the specified UART channel
 */
void initLogger(void);

/**
 * Output an error level message
 * @param message
 */
void error(char* message);

/**
 * Output a warning level message
 * @param message
 */
void warning(char* message);

/**
 * Output a debug level message
 * @param message
 */
void debug(char* message);


/**
 * Print out an array via debug in hex
 * @param array
 * @param length
 */
void debugArray(uint8_t* array, uint16_t length);


/**
 * Prints out a message with an integer. Helper method that saves some typing
 * @param message
 * @param number
 */
void debugInt(char* message, int64_t number);

/**
 * Output a info level message
 * @param message
 */
void info(char* message);

#endif
