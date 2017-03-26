/**
 * @file Logger.c
 * @author Chris Hajduk, Serj Babayan
 * @created August 24, 2014
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "./Logger.h"
#include "../Interfaces/UART.h"
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

static char logger_initialized = 0;

//general buffer. used for sprintf when printing out integers and such
static char* buffer;

void initLogger(void)
{
    initUART(LOGGER_UART_INTERFACE, LOGGER_UART_BAUD_RATE, LOGGER_BUFFER_INITIAL_LENGTH, LOGGER_BUFFER_MAX_LENGTH, UART_TX_ENABLE);
    logger_initialized = 1;
    buffer = malloc(100); //allocate 100 bytes for the buffer
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

void debugArray(uint8_t* array, uint16_t length)
{
    if (logger_initialized) {
        char* to_print = malloc(length*3 + 1);
        if (to_print == NULL){
            return;
        }
        int i;
        for (i = 0; i < length; i++) {
            sprintf(&to_print[i*3],"%02x ", array[i]);
        }
        to_print[length*3] = 0;

        writeMessage((char*)to_print, DEBUG_TAG_STRING, DEBUG_TAG_STRING_LENGTH);
        free(to_print);
    }
}

void debugInt(char* message, int64_t number){
    if (logger_initialized){
        sprintf(buffer, "%s: %lld", message, number);
        writeMessage(buffer, DEBUG_TAG_STRING, DEBUG_TAG_STRING_LENGTH);
    }
}

void info(char* message)
{
    if (logger_initialized) {
        writeMessage(message, INFO_TAG_STRING, INFO_TAG_STRING_LENGTH);
    }
}