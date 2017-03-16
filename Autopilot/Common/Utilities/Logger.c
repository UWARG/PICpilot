/**
 * @file Logger.c
 * @author Chris Hajduk, Serj Babayan
 * @created August 24, 2014
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "./Logger.h"
#include "../Interfaces/UART.h"

static char logger_initialized = 0;

void initLogger(void)
{
    initUART(LOGGER_UART_INTERFACE, LOGGER_UART_BAUD_RATE, LOGGER_BUFFER_INITIAL_LENGTH, LOGGER_BUFFER_MAX_LENGTH, UART_TX_ENABLE);
    logger_initialized = 1;
}

static void writeMessage(char* message, char* label, unsigned int label_length)
{
    //find length of the message
    unsigned int length = 0;
    while (message[length] != '\0') {
        length++;
    }

    //only send the uart data if theres enough space since we dont want to send over partial messages
    if (getTXSpace(LOGGER_UART_INTERFACE) >= label_length + 3 + length) {
        //add the message
        queueTXData(LOGGER_UART_INTERFACE, (unsigned char*) label, label_length);
        queueTXData(LOGGER_UART_INTERFACE, (unsigned char*) message, length);
        queueTXData(LOGGER_UART_INTERFACE, (unsigned char*) "\r\n\0", 3);
    }
}

void error(char* message)
{
    if (logger_initialized) {
        writeMessage(message, ERROR_TAG_STRING, ERROR_TAG_STRING_LENGTH);
    }
}

void warning(char* message)
{
    if (logger_initialized) {
        writeMessage(message, WARNING_TAG_STRING, WARNING_TAG_STRING_LENGTH);
    }
}

void debug(char* message)
{
    if (logger_initialized) {
        writeMessage(message, DEBUG_TAG_STRING, DEBUG_TAG_STRING_LENGTH);
    }
}

void info(char* message)
{
    if (logger_initialized) {
        writeMessage(message, INFO_TAG_STRING, INFO_TAG_STRING_LENGTH);
    }
}