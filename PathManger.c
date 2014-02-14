/*
 * File:   PathManager.c
 * Author: Chris Hajduk
 *
 * Created on February 4, 2014, 9:13 PM
 */

#include "main.h"
#include "PathManager.h"

#if !(PATH_MANAGER && ATTITUDE_MANAGER && COMMUNICATION_MANAGER)
#include "InterchipDMA.h"
#endif

#if DEBUG
#include <stdio.h>
#include <stdlib.h>
#include "UART1.h"
#endif

#if PATH_MANAGER

extern GPSData gpsData;
extern PMData pmData;

float k_gain = 1;

unsigned int currentIndex = 0;

PathData * path[100];

void pathManagerInit(void){
    //Communication with GPS
    init_SPI2();
    init_DMA0();

    //Interchip Communication
#if !ATTITUDE_MANAGER
    init_DMA1(); //Transmit Data on SPI1
    init_DMA2(); //Receive Data on SPI1
    init_SPI1();
#endif

    path[0] = initializePathNodeAndNext();
    path[1] = path[0]->next;

#if DEBUG
    InitUART1();
#endif
}

void pathManagerRuntime(void){
#if DEBUG
//    char str[16];
//    sprintf(&str,"%f",pmData.time);
//    UART1_SendString(&str);
#endif
    pmData.time = gpsData.time;


    //Orbit Following
    
    
    //Straight Path Following
    long double pathSlope = (path[currentIndex]->next->latitude - path[currentIndex]->latitude)/(path[currentIndex]->next->longitude - path[currentIndex]->longitude);
    //Using the first checkpoint as the origin

    float dDistance = sqrt(pow(path[currentIndex]->next->latitude * LATITUDE_TO_METERS - path[currentIndex]->latitude * LATITUDE_TO_METERS,2) + pow(path[currentIndex]->next->longitude * LONGITUDE_TO_METERS - path[currentIndex]->longitude * LONGITUDE_TO_METERS,2));
    float dAltitude = path[currentIndex]->next->altitude - path[currentIndex]->altitude;

    long double perpendicularXCoordinate = (gpsData.latitude + gpsData.longitude/pathSlope)/(pathSlope + 1/pathSlope);
    long double pathError = sqrt( pow( (perpendicularXCoordinate - gpsData.longitude) * LONGITUDE_TO_METERS, 2) + pow( (pathSlope * perpendicularXCoordinate - gpsData.latitude) * LATITUDE_TO_METERS, 2));

    pmData.sp_Heading = -atan(k_gain * pathError) * 180/PI; //Gain multiplied by distance from the target path

    pmData.sp_Altitude =  (float)(sqrt(pow(path[currentIndex]->next->latitude * LATITUDE_TO_METERS - gpsData.latitude * LATITUDE_TO_METERS,2) + pow(path[currentIndex]->next->longitude * LONGITUDE_TO_METERS - gpsData.longitude * LONGITUDE_TO_METERS,2)))/dDistance * dAltitude + path[currentIndex]->altitude;


}

unsigned int getIndexFromID(void){
    return 0;
}

PathData* initializePathNode(void){
    return (PathData *)malloc(sizeof(PathData));
}
PathData* initializePathNodeAndNext(void){
    PathData* temp = initializePathNode();
    temp->next = initializePathNode();
    return temp;
}

#endif