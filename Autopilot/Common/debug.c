/*
 * File:   debug.c
 *
 * Created on August 24, 2014, 05:38 PM
 */
#include "debug.h"

void initDebug(){
    InitUART1();
}

void error(char* message){
    unsigned int mLength = 0;
    //Check the entire string until you hit the null terminator
    //The buffer needs to have one array spot for the null terminator
    //The buffer also needs to take into account the concatenating string (length of 7 characters)
    while (message[mLength] != '\0' && mLength < DEBUG_BUFFER_LENGTH - ERROR_TAG_STRING_LENGTH - 1){
        mLength++;
    }

    //Compile the error message.
    char errorMessage[DEBUG_BUFFER_LENGTH];
    //Copy the first string
    unsigned int i;
    for (i = 0; i < ERROR_TAG_STRING_LENGTH; i++){
        errorMessage[i] = ERROR_TAG_STRING[i];
    }
    //Append the second string
    for (i = 0; i < mLength; i++){
        errorMessage[i + ERROR_TAG_STRING_LENGTH] = message[i];
    }
    errorMessage[i + ERROR_TAG_STRING_LENGTH] = '\r';
    errorMessage[i + ERROR_TAG_STRING_LENGTH + 1] = '\0';
    UART1_SendString(errorMessage);
}

void warning(char* message){
    unsigned int mLength = 0;
    //Check the entire string until you hit the null terminator
    //The buffer needs to have one array spot for the null terminator
    //The buffer also needs to take into account the concatenating string (length of 7 characters)
    while (message[mLength] != '\0' && mLength < DEBUG_BUFFER_LENGTH - WARNING_TAG_STRING_LENGTH - 1){
        mLength++;
    }

    //Compile the error message.
    char warningMessage[DEBUG_BUFFER_LENGTH];
    //Copy the first string
    unsigned int i;
    for (i = 0; i < WARNING_TAG_STRING_LENGTH; i++){
        warningMessage[i] = WARNING_TAG_STRING[i];
    }
    //Append the second string
    for (i = 0; i < mLength; i++){
        warningMessage[i + WARNING_TAG_STRING_LENGTH] = message[i];
    }
    warningMessage[i + WARNING_TAG_STRING_LENGTH] = '\r';
    warningMessage[i + WARNING_TAG_STRING_LENGTH + 1] = '\0';
    UART1_SendString(warningMessage);
}

void debug(char* message){
    unsigned int mLength = 0;
    //Check the entire string until you hit the null terminator
    //The buffer needs to have one array spot for the null terminator
    //The buffer also needs to take into account the concatenating string (length of 7 characters)
    while (message[mLength] != '\0' && mLength < DEBUG_BUFFER_LENGTH - DEBUG_TAG_STRING_LENGTH - 1){
        mLength++;
    }

    //Compile the error message.
    char debugMessage[DEBUG_BUFFER_LENGTH];
    //Copy the first string
    unsigned int i;
    for (i = 0; i < DEBUG_TAG_STRING_LENGTH; i++){
        debugMessage[i] = DEBUG_TAG_STRING[i];
    }
    //Append the second string
    for (i = 0; i < mLength; i++){
        debugMessage[i + DEBUG_TAG_STRING_LENGTH] = message[i];
    }
    debugMessage[i + DEBUG_TAG_STRING_LENGTH] = '\r';
    debugMessage[i + DEBUG_TAG_STRING_LENGTH + 1] = '\0';
    UART1_SendString(debugMessage);
}