/**
 * @file OrientationControl.h
 * @author Ian Frosst
 * @date March 2, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "main.h"

#ifndef ORIENTATIONCONTROL_H
#define	ORIENTATIONCONTROL_H

#define PID_CHANNELS 8 // to start

#define PID_RESET_TIME 1000 // timeout to reset I and D terms (ms)

#define MAX_ROLL_ANGLE 35.f // degrees
#define MAX_PITCH_ANGLE 35.f

#define MAX_ROLL_RATE 180.f // degrees/second
#define MAX_PITCH_RATE 180.f
#define MAX_YAW_RATE 240.f

#define wrap_180(x) (x < -180 ? x + 360 : (x > 180 ? x - 360 : x))

typedef struct { //holds values for a generic PID loop
    // Gains
    float Kp;
    float Ki;
    float Kd;
    
    uint32_t lastTime; // for derivative control
    float lastErr;  
    float lastDer; // last derivative, for filtering
    float integral;
    int16_t imax; // maximum value for integral
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

/**
 * Initializes a generic PID controller
 * @param pid Pointer to the PID_val struct
 * @param Kp
 * @param Ki
 * @param Kd
 * @param imax Limit for the integral term
 */
void initPID(PID_val* pid, float Kp, float Ki, float Kd, int16_t imax);

/**
 * Calculates output signal from a PID controller
 * @param pid Pointer to the PID_val struct to be updated
 * @param error Error value (setpoint - position)
 * @param scale Factor to help with I/O relationships
 * @return Control signal for a PID controller
 */
float PIDcontrol(PID_val* pid, float error, float scale);

/**
 * Initializes all the basic orientation PID controllers
 */
void orientationInit();

/**
 * Retrieves a pointer to one of the basic orientation PID controllers
 * @param channel Channel to retrieve
 * @return Pointer to PID_val struct for that channel
 */
PID_val* getPID(PID_channel channel);

/**
 * Gets a gain value from a PID channel
 * @param channel Channel to get gain from
 * @param type Type of gain (KP, KI or KD)
 * @return Requested gain value
 */
float getGain(PID_channel channel, PID_gain type);

/**
 * Sets a gain value for a PID channel
 * @param channel Channel to set gain for
 * @param type Type of gain (KP, KI or KD)
 * @param value Value to set the gain to
 */
void setGain(PID_channel channel, PID_gain type, float value);

uint8_t areGainsUpdated();
void forceGainUpdate();

#endif	/* ORIENTATIONCONTROL_H */
