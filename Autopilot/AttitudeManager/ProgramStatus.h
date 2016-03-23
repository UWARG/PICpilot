/* 
 * File:   ProgramStatus.h
 * Author: Chris Hajduk
 *
 * Created on March 21, 2016, 10:10 PM
 */

#ifndef PROGRAMSTATUS_H
#define	PROGRAMSTATUS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define NUM_SENSORS 2 //VectorNav, XBEE
#define VECTORNAV 0
#define XBEE 1


//Sensor Status Symbols
#define SENSOR_CONNECTED 0b00000001
#define SENSOR_INITIALIZED 0b00000010

//Program Status Symbols
#define INITIALIZATION 0
#define UNARMED 1
#define ARMING 2
#define MAIN_EXECUTION 3

/* Function Prototypes */

void setSensorStatus(char sensor, char status);
char getSensorStatus(char sensor);

void setProgramStatus(int status);
int getProgramStatus();

#ifdef	__cplusplus
}
#endif

#endif	/* PROGRAMSTATUS_H */

