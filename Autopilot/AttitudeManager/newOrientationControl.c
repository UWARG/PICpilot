/* 
 * File:   OrientationControl.c
 * Author: Ian Frosst
 *
 * Created on March 2, 2017, 3:29 PM
 */

#include "newOrientationControl.h"
#include "main.h"


/* 
 * Floating-point PID loops for attitude control. 
 * Inspired by ArduPilot control code
 * TODO: convert to fixed-point (integers) for increased speed
 * 
 */

#if 0 // removed for now
// Basic flight control signals
int sp_Throttle = 0;
int sp_RollRate = 0;
int sp_PitchRate = 0;
int sp_YawRate = 0;

// Abstracted flight control signals
int sp_Altitude = 0;
int sp_RollAngle = 0;
int sp_PitchAngle = 0;
int sp_Heading = 0;
#endif
// PID control values for all the loops
static PID_val pids[PID_CHANNELS];

static char gainsUpdated = 0; // updated gain flag


// To be called to initialize a new PID channel
void initPID(unsigned char channel, float Kp, float Ki, float Kd, long imax) {
    PID_val* pid = &pids[channel];
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->lastTime = 0;
    pid->lastErr = 0;
}

// PID loop function. error is (setpointValue - currentValue)
float PIDcontrol(unsigned char channel, float error) {
    float output = 0;
    
    PID_val* pid = &pids[channel];
    
    unsigned long now = getTime();
    unsigned int delta_msec = (now - pid->lastTime);
    
    // check if we've gone too long without updating (keeps the I and D from freaking out)
    if (delta_msec > PID_RESET_TIME || pid->lastTime == 0) {
        delta_msec = 0;
        pid->integral = 0;
        pid->lastErr = error;
    }
    
    pid->lastTime = now;

    output += pid->Kp * error; // Proportional control
    
    if (delta_msec > 0) { // only compute time-sensitive control if time has elapsed
        
        float dTime = delta_msec / 1000.0f; // elapsed time in seconds
        
        if (fabsf(pid->Ki) > 0) { // Integral control
            pid->integral += (pid->Ki * error) * dTime;
            
            if (pid->integral < -pid->imax) { // ensure integral stays manageable
                pid->integral = -pid->imax;
            } else if (pid->integral > pid->imax) {
                pid->integral = pid->imax;
            }
            output += pid->integral;
        }

        if (fabsf(pid->Kd) > 0) { // Derivative control
            float derivative = (error - pid->lastErr) / dTime;
            pid->lastErr = error;
            output += pid->Kd * derivative;
        }
    }
    
    return output;
}

float getGain(unsigned char channel, unsigned char type){
    if (channel < PID_CHANNELS) {
        if (type == GAIN_KP){
            return pids[channel].Kp;
        } else if (type == GAIN_KI){
            return pids[channel].Ki;
        } else if (type == GAIN_KD){
            return pids[channel].Kd;
        }
    }
    return -1;
}

void setGain(unsigned char channel, unsigned char type, float value){
    gainsUpdated = 1;
    if (channel < PID_CHANNELS) {
        if (type == GAIN_KP){
            pids[channel].Kp = value;
        } else if (type == GAIN_KI){
            pids[channel].Ki = value;
        } else if (type == GAIN_KD){
            pids[channel].Kd = value;
        }
    }
}

char areGainsUpdated(){
    if (gainsUpdated){
        gainsUpdated = 0;
        return 1;
    }
    return 0;
}

void forceGainUpdate(){
    gainsUpdated = 1;
}