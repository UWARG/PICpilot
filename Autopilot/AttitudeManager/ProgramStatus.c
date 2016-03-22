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
char programState;

void setSensorStatus(char sensor, char status){
    if (sensor < NUM_SENSORS){
        sensorState[sensor] = status;

#if DEBUG
        if (status && SENSOR_CONNECTED)
            debug("Sensor " + (sensor + 48) + " is connected");
        else if (status && SENSOR_INITIALIZED)
            debug("Sensor " + (sensor + 48) + " is initialized");
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
        return sensorState[sensor];
    }
    else{
    //Display Error Message
#if DEBUG
        warning("Invalid Sensor ID");
#endif
    }
}

void setProgramStatus(char status){
    programState = status;


#if DEBUG
    if (status == INITIALIZATION)
        debug("Attitude Manager Initialization");
    else if (status == UNARMED)
        debug("Vehicle is Unarmed. Waiting for arm.");
    else if (status == ARMING)
        debug("Arming: Motor Startup Procedure Started");
    else if (status == MAIN_EXECUTION){
        debug("Motor Startup Procedure Complete");
        debug("Autopilot Running...");
    }

#endif
}
char getProgramStatus(){
    return 0;
}
