/* 
 * File:   PathManger.h
 * Author: Chris Hajduk
 *
 * Created on February 4, 2014, 9:13 PM
 */

#ifndef PATHMANGER_H
#define	PATHMANGER_H

//Constants
#define LONGITUDE_TO_METERS 111302.61697430261
#define LATITUDE_TO_METERS 110574.61087757687


//Structs and typedefs

typedef struct _PathData{
    struct _PathData* previous;
    struct _PathData* next;
    long double longitude;
    long double latitude;
    float altitude;
    char id;
} PathData;


//Function Prototypes
void pathManagerInit(void);
void pathManagerRuntime(void);
PathData* initializePathNode(void);
PathData* initializePathNodeAndNext(void);


#endif	/* PATHMANGER_H */

