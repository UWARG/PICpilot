/**
 * @file Logger.c
 * @author Chris Hajduk, Serj Babayan
 * @created August 24, 2014
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "Logger.h"
#include "UART.h"
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

static char logger_initialized = 0;

void initLogger(void)
{
    initUART(LOGGER_UART_INTERFACE, LOGGER_UART_BAUD_RATE);
    logger_initialized = 1;
}

void debug(char* message)
{
     if (!logger_initialized) {
         return;
    }

    //find length of the message
    unsigned int length = 0;
    while (message[length] != '\0') {
        length++;
    }

    sendTXData(LOGGER_UART_INTERFACE, (unsigned char*) message, length);
    sendTXData(LOGGER_UART_INTERFACE, (unsigned char*) "\r\n", 2);
}

void debugN(char* message, uint16_t length)
{
     if (!logger_initialized) {
         return;
    }
     
    sendTXData(LOGGER_UART_INTERFACE, (unsigned char*) message, length);
    sendTXData(LOGGER_UART_INTERFACE, (unsigned char*) "\r\n", 2);
}