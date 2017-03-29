/* 
 * File:   ProgramStatus.h
 * Author: Chris Hajduk
 *
 * Created on March 21, 2016, 10:10 PM
 */

#ifndef PROGRAMSTATUS_H
#define	PROGRAMSTATUS_H

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
#define KILL_MODE_WARNING 4
#define KILL_MODE 5

/* Function Prototypes */

void setSensorStatus(char sensor, char status);
char getSensorStatus(char sensor);

void setProgramStatus(int status);
int getProgramStatus();

#endif	/* PROGRAMSTATUS_H */

