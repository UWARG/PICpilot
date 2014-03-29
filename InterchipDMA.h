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
//Data Structures

#if PATH_MANAGER || ATTITUDE_MANAGER
typedef struct _PMData {
    float time;     //4 Bytes   -  hhmmss.ssss
    long double latitude;  //8 Bytes - ddd.mmmmmm
    long double longitude; //8 Bytes - ddd.mmmmmm
    float altitude; // Meters
    float sp_Altitude; // Meters
    float heading;  //Degrees
    float sp_Heading; //Degrees
    float speed;    //KM/H
    char satellites;    //1 Byte
    char positionFix;   //0 = No GPS, 1 = GPS fix, 2 = DGSP Fix
} PMData;

typedef struct _AMData {
    WaypointWrapper waypoint;
    float pathGain;
    float orbitGain;
} AMData;
#endif

#if PATH_MANAGER
typedef struct _GPSData {
    long double latitude;  //8 Bytes
    long double longitude; //8 Bytes
    float time;     //4 Bytes
    float altitude;
    float heading;
    float speed;
    char satellites;    //1 Byte
    char positionFix;
} GPSData;
#endif

//Function Prototypes
void init_DMA0();
void init_DMA1();
void init_SPI1();

#if PATH_MANAGER
void init_SPI2();
void init_DMA2();
#endif

#endif	/* INTERCHIPDMA_H */

