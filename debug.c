/*
 * File:   debug.c
 *
 * Created on August 24, 2014, 05:38 PM
 */
#include <stdlib.h>
#include <string.h>
#include "StringUtils.h"
#include "UART1.h"

void error(char* message){
    //char* errorMessage = concat(concat("[error]", message),"/n");
    UART1_SendString(message);
    //free(errorMessage);
}

void warning(char* message){
    //char* warningMessage = concat(concat("[warning]", message),"/n");
    UART1_SendString(message);
    //free(warningMessage);
}

void debug(char* message){
    UART1_SendString(message);
}