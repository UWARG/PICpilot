/*
 * File:   Simulator.h
 * Author: WARG
 *
 * Created on April 13, 2016, 12:20 PM
 */

#ifndef SIMULATOR_H
#define	SIMULATOR_H
#include "../Common/UART1.h"
/*
 * File:   UART.h
 * Author: Mitch
 *
 * Created on June 15, 2013, 2:52 PM
 */
#define RAW_PACKET_BUFFER_SIZE 32
#define START_DELIMITER 0xAA
#define END_DELIMITER 0xBB

void setSimData(void);//struct ImuData *input);

char getRxPacketStatus();

struct GpsData *getSimData();

struct GpsData {
    unsigned char start;
    unsigned char pad1[3];
    long double latitude;
    long double longitude;
    float altitude;
    float heading;
    float speed;
    float time;

    unsigned char end;
    unsigned char pad2[3];
    // TODO: Add additional telemetry to be sent here
};

#endif	/* SIMULATOR_H */