/*
 * File:   Anaconda.c
 * Author: Chris Hajduk
 *
 * Created on July 2, 2015, 8:04 PM
 */

#include "DisplayQuad.h"
#include "PWM.h"
#include "AttitudeManager.h"
#include "delay.h"

char vehicleArmed = 0;

void initialization(){
    while (!vehicleArmed){
        imuCommunication();
        asm("CLRWDT");
        writeDatalink();
        readDatalink();
        inboundBufferMaintenance();
        outboundBufferMaintenance();
        Delay(200);
        asm("CLRWDT");
    }
}

void armVehicle(int delayTime){
#if DEBUG
    debug("MOTOR STARTUP PROCEDURE STARTED");
#endif
    asm("CLRWDT");
    Delay(delayTime);
    asm("CLRWDT");
    setPWM(1, MIN_PWM);
    setPWM(2, MIN_PWM);
    setPWM(3, MIN_PWM);
    setPWM(4, MIN_PWM);
    asm("CLRWDT");
    Delay(delayTime);
    asm("CLRWDT");
#if DEBUG
    debug("MOTOR STARTUP PROCEDURE COMPLETE");
#endif
}

void dearmVehicle(){
    int i = 1;
    for (i = 1; i <= NUM_CHANNELS; i++){
        setPWM(i, MIN_PWM);
    }
    while (!vehicleArmed){
        readDatalink();
        writeDatalink();
        inboundBufferMaintenance();
        outboundBufferMaintenance();
        Delay(200);
        asm("CLRWDT");
    }
}

//void takeOff(){
//
//}
//void landing(){
//
//}

void inputMixing(int* channels, int* rollRate, int* pitchRate, int* throttle, int* yawRate){
        if (getControlPermission(ROLL_CONTROL_SOURCE, ROLL_RC_SOURCE)){
            (*rollRate) = channels[0];
        }
        if (getControlPermission(THROTTLE_CONTROL_SOURCE, THROTTLE_RC_SOURCE))
            (*throttle) = (channels[2]);

        (*pitchRate) = channels[1];
        (*yawRate) = channels[3];
}

void outputMixing(int* channels, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw){
    
    channels[0] = (*control_Throttle) + (*control_Pitch) - (*control_Roll) - (*control_Yaw);// + MIN_PWM;  //Front
    channels[1] = (*control_Throttle) + (*control_Pitch) + (*control_Roll) + (*control_Yaw);// + MIN_PWM;   //Left
    channels[2] = (*control_Throttle) - (*control_Pitch) + (*control_Roll) - (*control_Yaw);// + MIN_PWM;  //Back
    channels[3] = (*control_Throttle) - (*control_Pitch) - (*control_Roll) + (*control_Yaw);// + MIN_PWM;   //Right
}

void checkLimits(int* channels){
    if (channels[0] > MAX_ROLL_PWM) {
        channels[0] = MAX_ROLL_PWM;
//        DEBUG("MAX ROLL limit reached %x." %channels[0]);
        // Limits the effects of the integrator, if the output signal is maxed out
//        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) > sp_RollRate - sp_ComputedRollRate) {
//            setIntegralSum(ROLL, getIntegralSum(ROLL)/1.1);
//        }
    }
    if (channels[0] < MIN_ROLL_PWM) {
        channels[0] = MIN_ROLL_PWM;
        char x[30];
        sprintf(&x, "MIN ROLL limit reached: %d", channels[0]);
        debug(&x);
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

void startArm()
{
    vehicleArmed = 1;
    armVehicle(2000);
}

void stopArm()
{
    vehicleArmed = 0;
    dearmVehicle();
}