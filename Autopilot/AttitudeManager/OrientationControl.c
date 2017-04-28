/**
 * @file OrientationControl.c
 * @author Ian Frosst
 * @date March 2, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "OrientationControl.h"
#include "main.h"

// PID control values for the basic loops
static PIDVal pids[CONTROL_CHANNELS];

static bool gainsUpdated = 0; // updated gain flag

// Initial PID gains. These are only used to keep sane values on startup.
const static float init_kp[CONTROL_CHANNELS] = {1, 1, 1, 1, 1, 1, 1, 1};
const static float init_ki[CONTROL_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const static float init_kd[CONTROL_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};

/*
 * Orientation-controller functions begin below here
 */

void orientationInit() {
    uint8_t i;
    for (i = 0; i < CONTROL_CHANNELS; i++) {
        initPID(&pids[i], init_kp[i], init_ki[i], init_kd[i], 1000);
    }
}

PIDVal* getPID(ControlChannel channel) {
    return &pids[channel];
}

float getGain(ControlChannel channel, GainType type){
    if (channel < CONTROL_CHANNELS) {
        if (type == KP){
            return pids[channel].kp;
        } else if (type == KI){
            return pids[channel].ki;
        } else if (type == KD){
            return pids[channel].kd;
        }
    }
    return -1; // TODO: return something better than a wrong value
}

void setGain(ControlChannel channel, GainType type, float value){
    gainsUpdated = 1;
    if (channel < CONTROL_CHANNELS) {
        if (type == KP){
            pids[channel].kp = value;
        } else if (type == KI){
            pids[channel].ki = value;
            pids[channel].integral = 0; // if we change the Ki (tuning), reset the integrator.
        } else if (type == KD){
            pids[channel].kd = value;
        }
    }
}

bool areGainsUpdated(){
    if (gainsUpdated){
        gainsUpdated = 0;
        return 1;
    }
    return 0;
}

void forceGainUpdate(){
    gainsUpdated = 1;
}
