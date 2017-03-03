/*
 * File:   Multirotor.c
 * Author: Ian Frosst
 *
 * Created on February 13, 2017, 8:58 PM
 */
#include "Multirotor.h"
#include "AttitudeManager.h"
#include "PWM.h"
#include "delay.h"
#include "ProgramStatus.h"

#if VEHICLE_TYPE == MULTIROTOR

int outputSignal[NUM_CHANNELS];
int control_Roll, control_Pitch, control_Yaw, control_Throttle;

void initialization(){
    setPWM(FRONT_LEFT_MOTOR, MIN_PWM);
    setPWM(FRONT_RIGHT_MOTOR, MIN_PWM);
    setPWM(BACK_RIGHT_MOTOR, MIN_PWM);
    setPWM(BACK_LEFT_MOTOR, MIN_PWM);
    
    int channel = 0;
    for (; channel < NUM_CHANNELS; channel++) {
        outputSignal[channel] = 0;
    }
    setProgramStatus(UNARMED);
    
    while (getProgramStatus() == UNARMED){
        StateMachine(STATEMACHINE_IDLE);
    }
}

void armVehicle(int delayTime){
    setProgramStatus(ARMING);
#if DEBUG
    debug("MOTOR STARTUP PROCEDURE STARTED");
#endif
    asm("CLRWDT");
    Delay(delayTime);
    asm("CLRWDT");
    setPWM(FRONT_LEFT_MOTOR, MIN_PWM);
    setPWM(FRONT_RIGHT_MOTOR, MIN_PWM);
    setPWM(BACK_RIGHT_MOTOR, MIN_PWM);
    setPWM(BACK_LEFT_MOTOR, MIN_PWM);
    asm("CLRWDT");
    Delay(delayTime);
    asm("CLRWDT");
#if DEBUG
    debug("MOTOR STARTUP PROCEDURE COMPLETE");
#endif
}

void dearmVehicle(){
    int i = 1;
    for (; i <= NUM_CHANNELS; i++){
        setPWM(i, MIN_PWM);
    }
    
    setProgramStatus(UNARMED);
    
    while (getProgramStatus() == UNARMED){
        StateMachine(STATEMACHINE_IDLE);
    }
    setProgramStatus(MAIN_EXECUTION);
}

void takeOff(){

}
void landing(){

}

void inputMixing(int* channelIn, int* rollRate, int* pitchRate, int* throttle, int* yawRate) { //no flaps on a quad
        if (getControlPermission(ROLL_CONTROL_SOURCE, RC_SOURCE, ROLL_CONTROL_SOURCE_SHIFT)){
            (*rollRate) = channelIn[ROLL_IN_CHANNEL - 1];
        }
        if (getControlPermission(THROTTLE_CONTROL_SOURCE, RC_SOURCE, THROTTLE_CONTROL_SOURCE_SHIFT)){
            (*throttle) = (channelIn[THROTTLE_IN_CHANNEL - 1]);
        }
        
        if (getControlPermission(PITCH_CONTROL_SOURCE, RC_SOURCE, PITCH_CONTROL_SOURCE_SHIFT)){
            (*pitchRate) = channelIn[PITCH_IN_CHANNEL - 1];
        }
        
        (*yawRate) = channelIn[YAW_IN_CHANNEL - 1];
}

void outputMixing(int* channelOut, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw){
    
    channelOut[FRONT_LEFT_MOTOR - 1] = (*control_Throttle) + (*control_Pitch) + (*control_Roll) - (*control_Yaw);// + MIN_PWM;  
    channelOut[FRONT_RIGHT_MOTOR - 1] = (*control_Throttle) + (*control_Pitch) - (*control_Roll) + (*control_Yaw);// + MIN_PWM;  
    channelOut[BACK_RIGHT_MOTOR - 1] = (*control_Throttle) - (*control_Pitch) - (*control_Roll) - (*control_Yaw);// + MIN_PWM; 
    channelOut[BACK_LEFT_MOTOR - 1] = (*control_Throttle) - (*control_Pitch) + (*control_Roll) + (*control_Yaw);// + MIN_PWM;  
}

void checkLimits(int* channelOut){
    int i = 0;
    for (i = 0; i < NUM_CHANNELS; i++) {
        if (channelOut[i] > MAX_PWM) {
            channelOut[i] = MAX_PWM;
        } else if (channelOut[i] < MIN_PWM) {
            channelOut[i] = MIN_PWM;
        }
    }
}

void highLevelControl(){
   
    uint8_t rollControlType = getControlValue(ROLL_CONTROL_TYPE, ROLL_CONTROL_TYPE_SHIFT);
    uint8_t rollControlSource = getControlValue(ROLL_CONTROL_SOURCE, ROLL_CONTROL_SOURCE_SHIFT);
    if (rollControlType == ANGLE_CONTROL) {
        setRollAngleSetpoint(getRollAngleInput(rollControlSource));
        setRollRateSetpoint(PIDcontrol(PID_ROLL_ANGLE, getRollAngleSetpoint() - getRoll()));
    } 
    else if (rollControlType == RATE_CONTROL) {
        setRollRateSetpoint(getRollRateInput(rollControlSource));
    }

    uint8_t pitchControlType = getControlValue(PITCH_CONTROL_TYPE, PITCH_CONTROL_TYPE_SHIFT);
    uint8_t pitchControlSource = getControlValue(PITCH_CONTROL_SOURCE, PITCH_CONTROL_SOURCE_SHIFT);
    if (pitchControlType == ANGLE_CONTROL) {
        setPitchAngleSetpoint(getPitchAngleInput(pitchControlSource));
        setPitchRateSetpoint(PIDcontrol(PID_PITCH_ANGLE, getPitchAngleSetpoint() - getPitch()));
    } 
    else if (pitchControlType == RATE_CONTROL) {
        setPitchRateSetpoint(getPitchRateInput(pitchControlSource));
    }
    
    if (getControlValue(HEADING_CONTROL, HEADING_CONTROL_SHIFT) == CONTROL_ON) { // if heading control is enabled
        setHeadingSetpoint(getHeadingInput(getControlValue(HEADING_CONTROL_SOURCE, HEADING_CONTROL_SOURCE_SHIFT))); // get heading value (GS or AP)
        setYawRateSetpoint(PIDcontrol(PID_HEADING, getHeadingSetpoint() - getHeading()));
    } 
    else {
        setYawRateSetpoint(getYawRateInput(RC_SOURCE));
    }
        
    if (getControlValue(ALTITUDE_CONTROL, ALTITUDE_CONTROL_SHIFT) == CONTROL_ON) { // if altitude control is enabled
        setAltitudeSetpoint(getAltitudeInput(getControlValue(ALTITUDE_CONTROL_SOURCE, ALTITUDE_CONTROL_SOURCE_SHIFT))); // get altitude value (GS or AP)
        setThrottleSetpoint(PIDcontrol(PID_ALTITUDE, getAltitudeSetpoint() - getAltitude()));
    } 
    else { // if no altitude control, get raw throttle input (RC or GS)
        setThrottleSetpoint(getThrottleInput(getControlValue(THROTTLE_CONTROL_SOURCE, THROTTLE_CONTROL_SOURCE_SHIFT)));
    }
}

void lowLevelControl() {  

    control_Roll = PIDcontrol(PID_ROLL_RATE, getRollRateSetpoint() - getRollRate());
    control_Pitch = PIDcontrol(PID_PITCH_RATE, getPitchRateSetpoint() - getPitchRate());
    control_Yaw = PIDcontrol(PID_YAW_RATE, getYawRateSetpoint() - getYawRate());
    control_Throttle = getThrottleSetpoint();
    
    //Mixing!
    outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);

    //Error Checking
    checkLimits(outputSignal);

    setAllPWM(outputSignal);
}

#endif