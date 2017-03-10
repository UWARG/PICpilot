/* 
 * File:   OrientationControl.h
 * Author: Ian Frosst
 *
 * Created on March 2, 2017, 3:29 PM
 */

#include "main.h"

#ifndef ORIENTATIONCONTROL_H
#define	ORIENTATIONCONTROL_H

#define PID_CHANNELS 8 // to start

#define PID_RESET_TIME 1000 // timeout to reset I and D terms (ms)

#define MAX_ROLL_ANGLE 35 // degrees
#define MAX_PITCH_ANGLE 35

#define MAX_ROLL_RATE 180 // degrees/second
#define MAX_PITCH_RATE 180
#define MAX_YAW_RATE 240

#define wrap_180(x) (x < -180 ? x + 360 : (x > 180 ? x - 360 : x))

typedef struct { //holds values for a generic PID loop
    // Gains
    float Kp;
    float Ki;
    float Kd;
    
    uint32_t lastTime; // for derivative control
    float lastErr;  
    float integral;
    float imax; // maximum value for integral
} PID_val;

typedef enum {
    KP = 0,
    KI,
    KD     
} PID_gain;

typedef enum {
    ROLL_RATE = 0,
    PITCH_RATE,
    YAW_RATE,
    ROLL_ANGLE,
    PITCH_ANGLE,
    HEADING,
    ALTITUDE,
    GSPEED
} PID_channel;

void initPID(PID_val* pid, float Kp, float Ki, float Kd, float imax);
void orientationInit();
PID_val* getPID(PID_channel channel);
float PIDcontrol(PID_val* pid, float error);
float getGain(PID_channel channel, PID_gain type);
void setGain(PID_channel channel, PID_gain type, float value);
uint8_t areGainsUpdated();
void forceGainUpdate();

#endif	/* ORIENTATIONCONTROL_H */
