/* 
 * File:   StringUtils.h
 * Author: Chris Hajduk
 *
 * Created on August 24, 2014, 6:23 PM
 */

#ifndef STRINGUTILS_H
#define	STRINGUTILS_H

#ifdef	__cplusplus
extern "C" {
#endif

// Includes
#include "main.h"
    
// Function Prototypes

/*****************************************************************************
 * Function: char* concat(char* str1, char* str2)
 *
 * Preconditions: None.
 *
 * Overview: Combines two strings (str1,str2) into a single string.
 *
 *
 * Input:   char* str1 -> The first string.
 *          char* str2 -> The second string.
 *
 * Output: The concatenated string.
 *
 *****************************************************************************/
char* concat(char* str1, char* str2);


#ifdef	__cplusplus
}
#endif

#endif	/* STRINGUTILS_H */

