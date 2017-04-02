/**
 * @file PID.h
 * @author Ian Frosst
 * @date  March 26, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef PID_H
#define	PID_H

#include "main.h"

#define PID_RESET_TIME 1000 // timeout to reset I and D terms (ms)

/* 
 * Floating-point PID loops for attitude control. 
 * Inspired by ArduPilot control code.
 * TODO: convert to fixed-point (integers) for increased speed, decreased size.
 * (could probably do with tenth- or hundredth-degree integers)
 * Floating-point math on the dsPIC33 is done via emulation, and is very slow.
 */

typedef struct { //holds values for a generic PID loop
    // Gains
    float kp;
    float ki;
    float kd;

    uint32_t last_time; // for derivative control
    float last_err;
    float last_der; // last derivative, for filtering
    float integral;
    int16_t i_max; // maximum value for integral
} PIDVal;

/**
 * Initializes a generic PID controller
 * @param pid Pointer to the PIDVal struct to initialize
 * @param Kp
 * @param Ki
 * @param Kd
 * @param imax Limit for the integral term
 */
void initPID(PIDVal* pid, float kp, float ki, float kd, int16_t i_max);

/**
 * Calculates output signal from a PID controller
 * @param pid Pointer to the PIDVal struct to be updated
 * @param error Error value (setpoint - position)
 * @param scale Factor to help with I/O relationships
 * @return Control signal for a PID controller
 */
float PIDcontrol(PIDVal* pid, float error, float scale);

#endif	/* PID_H */

