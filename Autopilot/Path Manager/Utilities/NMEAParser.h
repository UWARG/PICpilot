/**
 * @file NMEAParser.h
 * @author Serge Babayan
 * @date April 24, 2017
 * @brief Contains helper functions for parsing NMEA strings, which are generic
 * for a wide variety of GPS modules
 * 
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#ifndef NMEA_PARSER_H
#define	NMEA_PARSER

#include <stdint.h>
#include <stdbool.h>

/** NMEA header for a GGA string, the packet that provides positional information */
#define GGA_HEADER "GPGGA"

/** NMEA header for a VTG string, the packet that provides velocity and heading information */
#define VTG_HEADER "GPVTG"

/**
 * Verifies that the NMEA string is valid by using its checksum. Calculates the
 * checksum by doing a XOR across every byte
 * @param string The NMEA string to verify against. Note that to avoid the possibility
 * of an infinite loop, or going over memory, the max length of the string must also be passed
 * in
 * @return True if the given string is a valid NMEA string that meets the checksum
 * criteria, false otherwise
 */
bool isValidNMEAString(char* string, uint16_t max_length);

/**
 * Parses a GGA string and populates the provided parameters.
 * This message contains the following information:
 *  - latitude
 *  - longitude
 *  - time
 *  - altitude
 *  - position fix
 *  - satellites
 * Message structure:
 * $GPGGA,hhmmss.ss,Latitude,N,Longitude,E,FS,NoSV,HDOP,msl,m,Altref,m,DiffAge,DiffStation*cs<CR><LF>
 * for complete message details see
 * http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_%28GPS.G6-SW-10018%29.pdf
 * @param data The GGA string. Note that this shouldn't include the $ character,
 *  but should include the * character as it is used in the detection of the end of the string
 * @param latitude
 * @param longitude
 * @param utc_time
 * @param altitude
 * @param fix_status
 * @param num_satellites
 */
void parseGGA(char* data, long double* latitude, long double* longitude, float* utc_time, int16_t* altitude, uint8_t* fix_status, uint8_t* num_satellites);


/**
 * This message parses the "VTG" type NMEA message. This message contains
 *  the following information:
 *  - heading
 *  - speed
 *  Message structure:
 *  $GPVTG,cogt,T,cogm,M,sog,N,kph,K,mode*cs<CR><LF>
 * for complete message details see
 * http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_%28GPS.G6-SW-10018%29.pdf
 * @param data
 * @param speed
 * @param heading
 */
void parseVTG(char* data, float* speed, int16_t* heading);

#endif