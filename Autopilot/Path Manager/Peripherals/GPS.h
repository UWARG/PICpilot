/* 
 * File:   GPS.h
 * Author: serjb
 *
 * Created on April 23, 2017, 2:03 PM
 */

#ifndef GPS_H
#define	GPS_H

#include <stdint.h>

#define GPS_WARG_GPS 0
#define GPS_UBLOX_6 1

#define USE_GPS GPS_WARG_GPS

/** This is the structure that will be transmitted over interchip to the attitude manager*/
typedef struct{
    long double latitude;  //8 Bytes
    long double longitude; //8 Bytes
    float utc_time;     //4 Bytes. Time in seconds since 00:00 (midnight)
    float ground_speed; //in m/s
    int altitude; //in m
    int16_t heading; //in degrees. Should be between 0-360 at all times, but using integer just in case
    uint8_t num_satellites;    //1 Byte
    uint8_t fix_status; //0 = no fix, 1 = gps fix, 2 = differential gps fix (DGPS)
} GPSData;

/** Global variable representing the latest status info about gps coordinates. Should
  only be read from if the isNewGPSDataAvailable() function returns true */
extern GPSData gps_data;

/**
 * Initializes communications with the GPS
 */
void initGPS(void);

/**
 * Requests info from the GPS, and if applicabe, afterwhich the newest data should
 * be copied over to the gps_data struct
 */
void requestGPSInfo(void);

/**
 * Whether the module is connected or disconnected
 * @return
 */
bool isGPSConnected(void);

/**
 * If new GPS data is available to read
 */
bool isNewGPSDataAvailable(void);

#endif

