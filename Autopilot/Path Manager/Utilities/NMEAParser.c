/**
 * @file NMEAParser.c
 * @author Serge Babayan
 * @date April 24, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#include "../../Common/Common.h"
#include <stdbool.h>
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

static long double convertLatLong(char* lat_lon_string);
static byte asciiToHex(unsigned char asciiSymbol);

bool isValidNMEAString(char* string, uint16_t max_length){
    uint16_t i = 0;
    uint8_t checksum = 0;

    //we're not supposed to include $ in the checksum, so if the string we received
    //has it, make sure to skip it
    if (string[i] == '$'){
        i++;
    }
    
    while(string[i] != '*' && i < max_length){
        checksum ^= string[i];
        i++;
    }

    //if we exited the while loop because we reached the max length (therefore end of stirng never detected)
    if (i + 1 == max_length){
        return false;
    }

    //since rn i is at the '*' character, and not the checksum
    i++;

    //check to make sure the checksums are digits
    if (!isxdigit(string[i]) || !isxdigit(string[i+1])){
        return false;
    }
    
    //make sure the actual checksum is valid
    if (((checksum & 0xF0) >> 4) == asciiToHex(string[i]) && (checksum & 0x0F) == asciiToHex(string[i + 1])){
        return true;
    }
    
    return false;
}

void parseGGA(char* data, long double* latitude, long double* longitude, float* utc_time, int16_t* altitude, uint8_t* fix_status, uint8_t* num_satellites) {
    char values[15][20]; //store the value of the 15 components of this message (overkill, only 8 are needed)
    int i = 0;
    int j = 0;
    int n = 0;

    // fills the values array with information from the data[] variable
    for (i = 0; i < 15; i++) {
            n = 0;
            while (data[j] != ',' && data[j] != '\0') {
                    values[i][n] = data[j];
                    j++;
                    n++;
            }
            j++;
    }
    // fill the GPSData struct with info from values[]
    *latitude = convertLatLong(values[2]);
    if(values[3][0] == 'S') *latitude *= -1;
    *longitude = convertLatLong(values[4]);
    if(values[5][0] == 'W') *longitude *= -1;
    sscanf(values[1], "%f", utc_time);

    sscanf(values[9], "%d", altitude);
    sscanf(values[6], "%c", fix_status); //0 is no fix, 1 or 2 is valid fix, 6 is dead reckoning

    int temp;
    sscanf(values[7], "%d", &temp);//&GData->satellites); //Space Vehicles used, value between 0 and 12
    *num_satellites = temp & 0xFF;
}

void parseVTG(char* data, float* speed, int16_t* heading) {
	char values[10][20]; //store the value of the 10 components of this message (overkill, only 2 are needed)
	int i = 0;
	int j = 0;
	int n = 0;

	// fills the values array with information from the data[] variable
	for (i = 0; i < 10; i++) {
		n = 0;
		while (data[j] != ',' && data[j] != '\0') {
			values[i][n] = data[j];
			j++;
			n++;
		}
		j++;
	}
	// fill the GPSData struct with info from values[]
	sscanf(values[7], "%f", speed);
    if(values[1] != 0) sscanf(values[1], "%d", heading);
    
    *speed = *speed / 3.6;
}

/**
 * This function converts the NMEA formatted latitude or longitude string of
 * the format "DDMM.MMMMMM" (D = degrees, M = minutes) to simply degrees in
 * decimal form. The result is returned as a double. This function ignores
 * the global quadrant and thus all positions are > 0. The quadrant (sign) of
 * the coodrinate is parsed separately.
 * @param lat_lon_string
 * @return Double representation of the character lattiude/longitude
 */
static long double convertLatLong(char* lat_lon_string){

        //convert input string to double, divide by 100 to separate minutes from
        //degrees.
        double input = atof(lat_lon_string)/100;

        //convert the "minutes" portion of the above conversion to a decimal
        double minutes = ((input - (int)input)/60)*100;

	// Return degree value + fractional value of coordinate.
	return ((int)input + minutes);
}

/**
 * Converts a character string that respresents a hexadecimal into the corresponding
 * byte
 * @param asciiSymbol
 * @return
 */
static byte asciiToHex(unsigned char asciiSymbol) {
    char hexOut = 0;
    if (asciiSymbol == 0x2E)
        hexOut = 0x10;
    else if (asciiSymbol >= 0x30 && asciiSymbol <= 0x39){
        hexOut = asciiSymbol - 0x30;
    }
    else if (asciiSymbol >= 0x41 && asciiSymbol <= 0x46){
        hexOut = asciiSymbol - 0x37; //Letter "F"(ASCII 0x46) becomes 0xF
    }
    return hexOut;
}