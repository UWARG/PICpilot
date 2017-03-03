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
 * Inspired by ArduPilot control code.
 * TODO: convert to fixed-point (integers) for increased speed 
 * (could probably do with tenth- or hundredth-degree integers)
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

// PID control values for the basic loops
static PID_val pids[PID_CHANNELS];

static uint8_t gainsUpdated = 0; // updated gain flag

// Initial Values
#define YAW 0
#define PITCH   1
#define ROLL    2
#define HEADING 3
#define ALTITUDE 4
#define THROTTLE 5
#define FLAP 6

float kp_gain[7] = {1, 1.6, 4.8, 2, 1.5, 1, 1};//{1, 0.5, 2.5, 1.5, 1.25, 0.05};
float ki_gain[7]= {0, 0, 0, 0, 0, 0,0};
float kd_gain[7] = {3.5, 3.5, 3, 5, 0, 0, 0};

const static float init_kp[PID_CHANNELS] = {1};
const static float init_ki[PID_CHANNELS] = {0};
const static float init_kd[PID_CHANNELS] = {0};

// To be called to initialize a new PID channel
void initPID(PID_val* pid, float Kp, float Ki, float Kd, uint32_t imax) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->imax = imax;
    pid->integral = 0;
    pid->lastTime = 0;
    pid->lastErr = 0;
}

void orientationInit() {
    uint8_t i = 0;
    for (; i < PID_CHANNELS; i++) {
        initPID(&pids[i], init_kp[i], init_ki[i], init_kd[i], 0); // imax is 0 until we decide to use integral control
    }
}

// PID loop function. error is (setpointValue - currentValue)
float PIDcontrol(uint8_t channel, float error) {
    float output = 0;
    
    PID_val* pid = &pids[channel];
    
    uint32_t now = getTime();
    uint16_t delta_msec = (now - pid->lastTime);
    
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

float getGain(uint8_t channel, uint8_t type){
    if (channel < PID_CHANNELS) {
        if (type == GAIN_KP){
            return pids[channel].Kp;
        } else if (type == GAIN_KI){
            return pids[channel].Ki;
        } else if (type == GAIN_KD){
            return pids[channel].Kd;
        }
    }
    return -10000; // TODO: return something better than an obviously wrong value
}

void setGain(uint8_t channel, uint8_t type, float value){
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

uint8_t areGainsUpdated(){
    if (gainsUpdated){
        gainsUpdated = 0;
        return 1;
    }
    return 0;
}

void forceGainUpdate(){
    gainsUpdated = 1;
}
