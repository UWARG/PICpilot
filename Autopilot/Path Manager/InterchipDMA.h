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
#include "../Common/Common.h"

//Intercom pins
#define INTERCOM_1 PORTBbits.RB4 // input
#define INTERCOM_2 PORTBbits.RB5 // input
#define INTERCOM_3 PORTAbits.RA12 // output
#define INTERCOM_4 PORTAbits.RA13 // output


//Data Structures

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
char isDMADataAvailable();

void init_DMA2();

#endif	/* INTERCHIPDMA_H */

