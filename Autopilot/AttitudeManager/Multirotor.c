/*
 * File:   Multirotor.c
 * Author: Ian Frosst
 *
 * Created on February 13, 2017, 8:58 PM
 */

#include "PWM.h"
#include "AttitudeManager.h"
#include "delay.h"
#include "Multirotor.h"
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
        if (getControlPermission(ROLL_CONTROL_SOURCE, ROLL_RC_SOURCE, ROLL_CONTROL_SOURCE_SHIFT)){
            (*rollRate) = channelIn[ROLL_IN_CHANNEL - 1];
        }
        if (getControlPermission(THROTTLE_CONTROL_SOURCE, THROTTLE_RC_SOURCE, THROTTLE_CONTROL_SOURCE_SHIFT)){
            (*throttle) = (channelIn[THROTTLE_IN_CHANNEL - 1]);
        }
        
        if (getControlPermission(PITCH_CONTROL_SOURCE, PITCH_RC_SOURCE, PITCH_CONTROL_SOURCE_SHIFT)){
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
    //If the commands come from the ground station
    if (getControlPermission(ALTITUDE_CONTROL,ALTITUDE_CONTROL_ON,ALTITUDE_CONTROL_SHIFT) && getControlPermission(ALTITUDE_CONTROL_SOURCE,ALTITUDE_GS_SOURCE,ALTITUDE_CONTROL_SOURCE_SHIFT)) {setPitchAngleSetpoint(altitudeControl(getAltitudeInput(ALTITUDE_GS_SOURCE), getAltitude()));setAltitudeSetpoint(getAltitudeInput(ALTITUDE_GS_SOURCE));setThrottleSetpoint(throttleControl(getAltitudeInput(ALTITUDE_GS_SOURCE),getAltitude()));}
   //If the commands come from the autopilot
    else if (getControlPermission(ALTITUDE_CONTROL,ALTITUDE_CONTROL_ON,ALTITUDE_CONTROL_SHIFT) && getControlPermission(ALTITUDE_CONTROL_SOURCE,ALTITUDE_AP_SOURCE,ALTITUDE_CONTROL_SOURCE_SHIFT)) {setPitchAngleSetpoint(altitudeControl(getAltitudeInput(ALTITUDE_AP_SOURCE), getAltitude()));setAltitudeSetpoint(getAltitudeInput(ALTITUDE_AP_SOURCE));setThrottleSetpoint(throttleControl(getAltitudeInput(ALTITUDE_AP_SOURCE),getAltitude()));}
    //If commands come from the ground station
    else if (getControlPermission(PITCH_CONTROL_SOURCE, PITCH_GS_SOURCE,PITCH_CONTROL_SOURCE_SHIFT)) setPitchAngleSetpoint(getPitchAngleInput(PITCH_GS_SOURCE));
    //If commands come from the RC controller
    else setPitchAngleSetpoint(getPitchAngleInput(PITCH_RC_SOURCE));

    //If commands come from the autopilot -//TODO:ADD heading autopilt source
    if (getControlPermission(HEADING_CONTROL, HEADING_CONTROL_ON, HEADING_CONTROL_SHIFT) && getControlPermission(HEADING_CONTROL_SOURCE,HEADING_GS_SOURCE,HEADING_CONTROL_SOURCE_SHIFT)) {setRollAngleSetpoint(headingControl(getHeadingInput(HEADING_GS_SOURCE), getHeading()));}
    //If the commands come from the autopilot
    else if (getControlPermission(HEADING_CONTROL,HEADING_CONTROL_ON,HEADING_CONTROL_SHIFT) && getControlPermission(HEADING_CONTROL_SOURCE,HEADING_AP_SOURCE,HEADING_CONTROL_SOURCE_SHIFT)) {setRollAngleSetpoint(headingControl(getHeadingInput(HEADING_AP_SOURCE), getHeading()));setHeadingSetpoint(getHeadingInput(HEADING_AP_SOURCE));}
    //If commands come from the ground station
    else if (getControlPermission(ROLL_CONTROL_SOURCE, ROLL_GS_SOURCE, ROLL_CONTROL_SOURCE_SHIFT)) {setRollAngleSetpoint(getRollAngleInput(ROLL_GS_SOURCE));}
    //If commands come from the RC controller
    else  {setRollAngleSetpoint(getRollAngleInput(ROLL_RC_SOURCE));}
}

void lowLevelControl() {
    //If commands come from the autopilot
    if (getControlPermission(ROLL_CONTROL_TYPE, ANGLE_CONTROL,ROLL_CONTROL_TYPE_SHIFT) || getControlPermission(HEADING_CONTROL,HEADING_CONTROL_ON, HEADING_CONTROL_SHIFT)) {
        setRollRateSetpoint(rollAngleControl(getRollAngleSetpoint(), -getRoll()));       //Keep a steady Roll Angle
    }
    //If commands come from the ground station
    else if (getControlPermission(ROLL_CONTROL_SOURCE, ROLL_GS_SOURCE,ROLL_CONTROL_SOURCE_SHIFT) || getControlPermission(ALTITUDE_CONTROL,ALTITUDE_CONTROL_ON,0)) {
        setRollRateSetpoint(getRollRateInput(ROLL_GS_SOURCE));
    }
    //If commands come from the RC Controller
    else {
        setRollRateSetpoint(getRollRateInput(ROLL_RC_SOURCE));
    }

    //If commands come from the autopilot
    if (getControlPermission(PITCH_CONTROL_TYPE, ANGLE_CONTROL,PITCH_CONTROL_TYPE_SHIFT)){
        setPitchRateSetpoint(pitchAngleControl(getPitchAngleSetpoint(), -getPitch()));   //Keep a steady Pitch Angle
        setPitchRateSetpoint(coordinatedTurn(getPitchRateSetpoint(), getRoll()));       //Apply Coordinated Turn
    }
    //If commands come from the ground station
    else if (getControlPermission(PITCH_CONTROL_SOURCE, PITCH_GS_SOURCE,PITCH_CONTROL_SOURCE_SHIFT)){
        setPitchRateSetpoint(getPitchRateInput(PITCH_GS_SOURCE));                         //Keep a steady Pitch Angle
        setPitchRateSetpoint(coordinatedTurn(getPitchRateSetpoint(), getRoll()));       //Apply Coordinated Turn
    }
    //If commands come from the RC Controller
    else{
        setPitchRateSetpoint(getPitchRateInput(PITCH_RC_SOURCE));                         //Keep a steady Pitch Angle
        setPitchRateSetpoint(coordinatedTurn(getPitchRateSetpoint(), getRoll()));       //Apply Coordinated Turn
    }
    
    //If commands come from the ground station
    if (getControlPermission(THROTTLE_CONTROL_SOURCE, THROTTLE_GS_SOURCE ,THROTTLE_CONTROL_SOURCE_SHIFT)) {
        setThrottleSetpoint(getThrottleInput(THROTTLE_GS_SOURCE));
    }
    //If commands come from the RC Controller
    else if (getControlPermission(THROTTLE_CONTROL_SOURCE,THROTTLE_RC_SOURCE, THROTTLE_CONTROL_SOURCE_SHIFT)) {
        setThrottleSetpoint(getThrottleInput(THROTTLE_RC_SOURCE));
    }

    control_Roll = rollRateControl((float)getRollRateSetpoint(), -getRollRate());
    control_Pitch = pitchRateControl((float)getPitchRateSetpoint(), -getPitchRate());
    control_Yaw = yawRateControl((float)getYawRateSetpoint(), -getYawRate());
    control_Throttle = getThrottleSetpoint();
    

//    control_Throttle = throttleControl(getAltitudeInput(ALTITUDE_AP_SOURCE), getAltitude());       //Hold a steady throttle (more or less airspeed for fixed wings)
//    setRollRateSetpoint(rollAngleControl(getRollAngleSetpoint(), getRoll()));       //Keep a steady Roll Angle
//    setPitchRateSetpoint(pitchAngleControl(getPitchAngleSetpoint(), getPitch()));   //Keep a steady Pitch Angle
//    control_Roll = rollRateControl(getRollRateSetpoint(), getRollRate());
//    control_Pitch = pitchRateControl(getPitchRateSetpoint(), getPitchRate());
//    control_Yaw = yawRateControl(getYawRateSetpoint(), getYawRate());
//
    //Mixing!
    outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);

    //Error Checking
    checkLimits(outputSignal);

    setAllPWM(outputSignal);
}

#endif