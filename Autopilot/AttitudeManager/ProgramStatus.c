/*
 * File:   ProgramStatus.c
 * Author: Chris Hajduk
 *
 * Created on March 21, 2016, 3:40 PM
 */
#include "main.h"
#include "ProgramStatus.h"
#include "../Common/debug.h"

char sensorState[NUM_SENSORS];
int programState;

void setSensorStatus(char sensor, char status){
    if (sensor < NUM_SENSORS){
        sensorState[(int)sensor] = status;

#if DEBUG
        char str[20];
        if (status & SENSOR_CONNECTED){
            sprintf(str,"Sensor %d is connected", sensor);
            debug(str);
        }
        else if (status & SENSOR_INITIALIZED){
            sprintf(str, "Sensor %d is initialized", sensor);
            debug(str);
        }
#endif
    }
    else{
    //Display Error Message
#if DEBUG
        warning("Invalid Sensor ID");
#endif
    }
}

char getSensorStatus(char sensor){
    if (sensor < NUM_SENSORS){
        return sensorState[(int)sensor];
    }
    else{
    //Display Error Message
#if DEBUG
        warning("Invalid Sensor ID");
#endif
        return -1;
    }
}

void setProgramStatus(int status){
    programState = status;

#if DEBUG
    if (status == INITIALIZATION) {
        debug("Attitude Manager Initialization");
    } else if (status == UNARMED) {
        debug("Vehicle is Unarmed. Waiting for arm.");
    } else if (status == ARMING) {
        debug("Arming: Motor Startup Procedure Started");
    } else if (status == MAIN_EXECUTION) {
        debug("Motor Startup Procedure Complete");
        debug("Autopilot Running...");
    } else if (status == KILL_MODE_WARNING) {
        debug("I am contemplating suicide.");
    } else if (status == KILL_MODE) {
        debug("I am attempting to destroy myself now.");
    }

#endif
}
int getProgramStatus(){
    return programState;
}
