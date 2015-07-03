/*
 * File:   Anaconda.c
 * Author: Chris Hajduk
 *
 * Created on July 2, 2015, 8:04 PM
 */

#include "Anaconda.h"

void inputMixing(int* channels, int* rollRate, int* pitchRate, int* throttle, int* yawRate){
        if ((ROLL_CONTROL_SOURCE & controlLevel) == 0){
            rollRate = channels[0];
        }
        if ((PITCH_CONTROL_SOURCE & controlLevel) == 0){
            pitchRate = channels[1];
        }
        if ((THROTTLE_CONTROL_SOURCE & controlLevel) == 0)
            throttle = (channels[2]);


        #if(TAIL_TYPE == STANDARD_TAIL)
        {
            if ((PITCH_CONTROL_SOURCE & controlLevel) == 0)
            {
            pitchRate = channels[1];
            }
            yawRate = channels[3];
        }
        #endif
        #if(TAIL_TYPE == INV_V_TAIL)
        {
            if ((PITCH_CONTROL_SOURCE & controlLevel) == 0)
            {
                pitchRate = (channels[1] - channels[3]) / (2 * ELEVATOR_PROPORTION);
            }
            yawRate = (channels[1] + channels[3] ) / (2 * RUDDER_PROPORTION);
        }
        #endif
}

void outputMixing(int* channels, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw){
    //begin code for different tail configurations
    #if(TAIL_TYPE == STANDARD_TAIL)    //is a normal t-tail
    {
        channels[1] = control_Pitch
        channels[3] = control_Yaw
    }

    #elif(TAIL_TYPE == V_TAIL)    //V-tail
    {
        //place holder
    }

    #elif(TAIL_TYPE == INV_V_TAIL)    //Inverse V-Tail
    {
        channels[1] =  control_Yaw * RUDDER_PROPORTION + control_Pitch * ELEVATOR_PROPORTION ; //Tail Output Right
        channels[3] =  control_Yaw * RUDDER_PROPORTION - control_Pitch * ELEVATOR_PROPORTION ; //Tail Output Left
    }
    #endif
    channels[0] = control_Roll;
    channels[2] = control_Throttle;
}

void checkLimits(int* channels){
    if (channels[0] > MAX_ROLL_PWM) {
        channels[0] = MAX_ROLL_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
//        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) > sp_RollRate - sp_ComputedRollRate) {
//            setIntegralSum(ROLL, getIntegralSum(ROLL)/1.1);
//        }
    }
    if (channels[0] < MIN_ROLL_PWM) {
        channels[0] = MIN_ROLL_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
//        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) < sp_RollAngle - sp_ComputedRollRate) {
//            setIntegralSum(ROLL, getIntegralSum(ROLL)/1.1);
//        }
    }
    if (channels[1] > MAX_PITCH_PWM) {
        channels[1] = MAX_PITCH_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
//        if (getIntegralSum(PITCH) * getGain(PITCH, GAIN_KI) > sp_PitchAngle - sp_ComputedPitchRate) {
//            setIntegralSum(PITCH, getIntegralSum(PITCH)/1.1);
//        }
    }
    else if (channels[1] < MIN_PITCH_PWM) {
        channels[1] = MIN_PITCH_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
//        if (getIntegralSum(PITCH) * getGain(PITCH, GAIN_KI) < sp_PitchAngle - sp_ComputedPitchRate) {
//            setIntegralSum(PITCH, getIntegralSum(PITCH)/1.1);
//        }
    }
    //Throttle = 2
    if (channels[2] > MAX_PWM){
        channels[2] = MAX_PWM;
    }
    else if (channels[2] < MIN_PWM){
        channels[2] = MIN_PWM;
    }

    if (channels[3] > MAX_YAW_PWM)
        channels[3] = MAX_YAW_PWM;
    else if (channels[3] < MIN_YAW_PWM)
        channels[3] = MIN_YAW_PWM;
}