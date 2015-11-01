/* 
 * File:   debug.h
 * Author: Chris Hajduk
 *
 * Created on August 24, 2014, 6:02 PM
 */

#ifndef DEBUG_H
#define	DEBUG_H

#ifdef	__cplusplus
extern "C" {
#endif
    
// Includes
#include "../Common/UART1.h"

// 50 character debug strings
#define DEBUG_BUFFER_LENGTH 50
//Debug character prefixes
#define DEBUG_TAG_STRING "[debug]"
#define DEBUG_TAG_STRING_LENGTH 7
#define ERROR_TAG_STRING "[error]"
#define ERROR_TAG_STRING_LENGTH 7
#define WARNING_TAG_STRING "[warning]"
#define WARNING_TAG_STRING_LENGTH 9

// Function Prototypes    
    /*****************************************************************************
 * Function: void initDebug(void)
 *
 * Preconditions: None.
 *
 * Overview: Initializes the UART1 port for debugging purposes.
 * 
 *
 * Input: None.
 *
 * Output: None.
 *
 *****************************************************************************/
void initDebug(void);

/*****************************************************************************
 * Function: void error(char* message)
 *
 * Preconditions: UART1 must have been initialized.
 *
 * Overview: Provides an error message over the debugging UART port. Prefixes
 *  the message with a "[error]".
 * 
 *
 * Input: char* message -> The message associated with the error.
 *
 * Output: None.
 *
 *****************************************************************************/
void error(char* message);

/*****************************************************************************
 * Function: void warning(char* message)
 *
 * Preconditions: UART1 must have been initialized.
 *
 * Overview: Provides an warning message over the debugging UART port. Prefixes
 *  the message with a "[warning]".
 *
 *
 * Input: char* message -> The message associated with the warning.
 *
 * Output: None.
 *
 *****************************************************************************/
void warning(char* message);

/*****************************************************************************
 * Function: void debug(char* message)
 *
 * Preconditions: UART1 must have been initialized.
 *
 * Overview: Sends a message to the the debug UART port.
 *
 *
 * Input: char* message -> The message.
 *
 * Output: None.
 *
 *****************************************************************************/
void debug(char* message);



#ifdef	__cplusplus
}
#endif

#endif	/* DEBUG_H */

