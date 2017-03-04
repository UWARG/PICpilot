/* 
 * File:   OrientationControl.h
 * Author: Ian Frosst
 *
 * Created on March 2, 2017, 3:29 PM
 */

#include "main.h"

#ifndef ORIENTATIONCONTROL_H
#define	ORIENTATIONCONTROL_H

#define GAIN_KP 0
#define GAIN_KI 1
#define GAIN_KD 2

#define PID_CHANNELS 8 // to start

#define PID_ROLL_RATE   0
#define PID_PITCH_RATE  1
#define PID_YAW_RATE    2

#define PID_ROLL_ANGLE  3
#define PID_PITCH_ANGLE 4

#define PID_HEADING     5
#define PID_ALTITUDE    6
#define PID_SPEED       7 //Airspeed or groundspeed? discuss.

#define PID_RESET_TIME 1000 // timeout to reset I and D terms (ms)

#define MAX_ROLL_ANGLE 35.f // degrees
#define MAX_PITCH_ANGLE 35.f

#define MAX_ROLL_RATE 180.f // degrees/second
#define MAX_PITCH_RATE 180.f
#define MAX_YAW_RATE 240.f

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

typedef struct PID_val { //holds values for a generic PID loop
    // Gains
    float Kp;
    float Ki;
    float Kd;
    
    uint32_t lastTime; // for derivative control
    float lastErr;  
    float integral;
    uint32_t imax; // maximum value for integral
} PID_val;


void initPID(PID_val* pid, float Kp, float Ki, float Kd, uint32_t imax);
void orientationInit();
float PIDcontrol(uint8_t channel, float error);
float getGain(uint8_t channel, uint8_t type);
void setGain(uint8_t channel, uint8_t type, float value);
uint8_t areGainsUpdated();
void forceGainUpdate();

#endif	/* ORIENTATIONCONTROL_H */
