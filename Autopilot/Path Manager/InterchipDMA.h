/*
 * File:   InterchipDMA.h
 * Author: Chris Hajduk
 *
 * Created on February 2, 2014, 2:10 PM
 */

//TODO: Clean up this H file and the corresponding C file

#ifndef INTERCHIPDMA_H
#define	INTERCHIPDMA_H

#include "main.h"
#include "PathManager.h"

//Intercom pins
#define INTERCOM_1 PORTBbits.RB4 // input
#define INTERCOM_2 PORTBbits.RB5 // input
#define INTERCOM_3 PORTAbits.RA12 // output
#define INTERCOM_4 PORTAbits.RA13 // output


//Data Structures

typedef struct _PMData {
    float time;     //4 Bytes   -  hhmmss.ssss
    long double latitude;  //8 Bytes - ddd.mmmmmm
    long double longitude; //8 Bytes - ddd.mmmmmm
    float speed;    //KM/H
    float altitude;
    int sp_Altitude; // Meters
    int heading;  //Degrees
    int sp_Heading; //Degrees
    char satellites;    //1 Byte
    char positionFix;   //0 = No GPS, 1 = GPS fix, 2 = DGSP Fix
    char targetWaypoint;
    char waypointCount;
    char batteryLevel;
    char checkbyteDMA;
} PMData;

typedef struct _AMData {
    WaypointWrapper waypoint;
    float pathGain;
    float orbitGain;
    float calibrationHeight;
    char command;
    char checksum;
    char checkbyteDMA;
    char padding;
} AMData;

typedef struct _GPSData {
    long double latitude;  //
    long double longitude; //
    float time;     //4 Bytes
    float speed;
    int altitude;
    int heading;
    char satellites;    //1 Byte
    char positionFix;
} GPSData;

//Function Prototypes
void init_DMA0();
void init_DMA1();
void init_SPI1();
char isDMADataAvailable();

void init_SPI2();
void init_DMA2();

#endif	/* INTERCHIPDMA_H */

