/**
 * @file Utilities.c
 * @author Serj Babayan
 * @date April 23, 2017, 3:38AM
 * 
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#ifndef UTILITIES_H
#define	UTILITIES_H

/**
 * Converts a byte value to its character hex equivalent. Note that for this function to work
 * properly, you only want to ever give it half a byte (so up to 16)
 * @param checkSumHalf
 * @return The hex string value of the byte
 */
char byteToHexString(unsigned int checkSumHalf);

/**
 * Converts an ascii character into its hex/byte equivalent value
 * @param asciiSymbol
 * @return
 */
char asciiToHex(unsigned char asciiSymbol);

#endif

