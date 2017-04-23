/**
 * @file GPS.h
 * @author Serj Babayan
 * @created April 23, 2017, 1:57 AM
 * Wrapper for the gps module. Contains the data struct that will be transmitted over
 */

#ifndef GPS_H
#define	GPS_H

/**
 * This is the struct that will be transmitted over the SPI and I2C busses
 */
typedef struct{
    long double latitude;  //8 Bytes
    long double longitude; //8 Bytes
    uint32_t utc_time;     //4 Bytes. Time in seconds since 00:00 (midnight)
    float ground_speed; //in m/s
    float altitude; //in m
    int16_t heading; //in degrees. Should be between 0-360 at all times, but using integer just in case
    uint8_t num_satellites;    //1 Byte
    uint8_t fix_status; //0 = no fix, 1 = gps fix, 2 = differential gps fix (DGPS)
    uint8_t connected; //will always output 1 to indicate that the module is in fact connected
} GPSData;

/** Global reference to the GPS data that will be updated once new data is parsed */
extern GPSData gps_data;

/**
 * Configures the GPS module for proper hz settings and dgps
 */
void configureGPS(void);

/**
 * If applicable, reads and parses the data in the UART buffer. If valid data
 * was received, updates the global gps struct and sets the data available flag
 * to true
 */
void parseIncomingGPSData(void);

/**
 * If new gps data is available to read
 */
void isNewDataAvailable(void);

#endif

