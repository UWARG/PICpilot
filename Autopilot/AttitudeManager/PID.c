/**
 * @file PID.c
 * @author Ian Frosst
 * @date  March 26, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "PID.h"
#include "../Common/Clock/Timer.h"

/**
 * Filtering constant for derivative. Between 0 and 1.
 * The larger this is, the more twitchy D control is. 
 */
#define FILTER 0.4f

/* Generic PID functions. Can be used to PID other things (flaps, etc) */

// To be called to initialize a new PID channel
void initPID(PIDVal* pid, float kp, float ki, float kd, int16_t i_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->integral = 0;
    pid->last_time = 0;
    pid->last_err = 0;
    pid->last_der = 0;
}

// PID loop function. error is (setpointValue - currentValue)
float PIDcontrol(PIDVal* pid, float error, float scale) {
    float output = 0;
        
    uint64_t now = getTimeUs();
    uint32_t delta_usec = (now - pid->last_time);
    
    // check if we've gone too long without updating (keeps the I and D from freaking out)
    if (delta_usec > PID_RESET_TIME || pid->last_time == 0) {
        delta_usec = 0;
        pid->integral = 0;
        pid->last_err = error;
    }
    
    pid->last_time = now;

    output += pid->kp * error; // Proportional control
    
    if (delta_usec > 500) { // only compute time-sensitive control if time has elapsed (more then 500 us)
        float dTime = delta_usec / 1e6f; // elapsed time in seconds
        
        if (fabsf(pid->ki) > 0) { // Integral control
            pid->integral += (pid->ki * error) * dTime;
            
            if (pid->integral < -pid->i_max) { // ensure integral stays manageable
                pid->integral = -pid->i_max;
            } else if (pid->integral > pid->i_max) {
                pid->integral = pid->i_max;
            }
            output += pid->integral;
        } else {
            pid->integral = 0; // if there's no Ki, reset the integrator (makes tuning easier)
        }

        if (fabsf(pid->kd) > 0) { // Derivative control
            float derivative = (error - pid->last_err) / dTime;
            derivative = derivative * FILTER + pid->last_der * (1-FILTER); // reduce jitter in derivative by averaging
            pid->last_err = error;
            pid->last_der = derivative;
            output += pid->kd * derivative;
        }
    }
    
    return output * scale;
}

