/**
 * @file   Multirotor.c
 * @author Ian Frosst
 * @date February 13, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "Multirotor.h"
#include "AttitudeManager.h"
#include "PWM.h"
#include "delay.h"
#include "ProgramStatus.h"

#if VEHICLE_TYPE == MULTIROTOR

static int outputSignal[NUM_CHANNELS];
static int control_Roll, control_Pitch, control_Yaw, control_Throttle;

void initialization(){
    setPWM(1, MIN_PWM);
    setPWM(2, MIN_PWM);
    setPWM(3, MIN_PWM);
    setPWM(4, MIN_PWM);
    
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
    for (; i <= NUM_CHANNELS; i++){
        setPWM(i, MIN_PWM);
    }
    
    setProgramStatus(UNARMED);
    
    while (getProgramStatus() == UNARMED){
        StateMachine(STATEMACHINE_IDLE);
    }
    setProgramStatus(MAIN_EXECUTION);
}

void inputMixing(int* channelIn, int* rollRate, int* pitchRate, int* throttle, int* yawRate) { //no flaps on a quad
    if (getControlValue(ROLL_CONTROL_SOURCE) == RC_SOURCE){
        (*rollRate) = channelIn[ROLL_IN_CHANNEL - 1];
    }
    if (getControlValue(THROTTLE_CONTROL_SOURCE) == RC_SOURCE){
        (*throttle) = channelIn[THROTTLE_IN_CHANNEL - 1];
    }
    if (getControlValue(PITCH_CONTROL_SOURCE) == RC_SOURCE){
        (*pitchRate) = channelIn[PITCH_IN_CHANNEL - 1];
    }

    (*yawRate) = channelIn[YAW_IN_CHANNEL - 1];
}

void outputMixing(int* channelOut, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw){
# if ROTOR_TYPE == QUAD_X
    channelOut[FRONT_LEFT_MOTOR - 1] = (*control_Throttle) + (*control_Pitch) + (*control_Roll) - (*control_Yaw);
    channelOut[FRONT_RIGHT_MOTOR - 1] = (*control_Throttle) + (*control_Pitch) - (*control_Roll) + (*control_Yaw);
    channelOut[BACK_RIGHT_MOTOR - 1] = (*control_Throttle) - (*control_Pitch) - (*control_Roll) - (*control_Yaw);
    channelOut[BACK_LEFT_MOTOR - 1] = (*control_Throttle) - (*control_Pitch) + (*control_Roll) + (*control_Yaw);
#elif ROTOR_TYPE == QUAD_P
    channelOut[FRONT_MOTOR - 1] = (*control_Throttle) + (*control_Pitch) - (*control_Yaw);
    channelOut[RIGHT_MOTOR - 1] = (*control_Throttle) - (*control_Roll) + (*control_Yaw);
    channelOut[BACK_MOTOR - 1] = (*control_Throttle) - (*control_Pitch) - (*control_Yaw);
    channelOut[LEFT_MOTOR - 1] = (*control_Throttle) + (*control_Roll) + (*control_Yaw);

#endif
}

void checkLimits(int* channelOut){
    int i;
    for (i = 0; i < NUM_CHANNELS; i++) {
        channelOut[i] = constrain(channelOut[i], MIN_PWM, MAX_PWM);
    }
}

void highLevelControl(){
   
    uint8_t rollControlType = getControlValue(ROLL_CONTROL_TYPE);
    uint8_t rollControlSource = getControlValue(ROLL_CONTROL_SOURCE);
    if (rollControlType == ANGLE_CONTROL) {
        setRollAngleSetpoint(getRollAngleInput(rollControlSource));
        setRollRateSetpoint(PIDcontrol(getPID(ROLL_ANGLE), getRollAngleSetpoint() - getRoll(), MAX_ROLL_RATE / MAX_ROLL_ANGLE));
    } 
    else if (rollControlType == RATE_CONTROL) {
        setRollRateSetpoint(getRollRateInput(rollControlSource));
    }

    uint8_t pitchControlType = getControlValue(PITCH_CONTROL_TYPE);
    uint8_t pitchControlSource = getControlValue(PITCH_CONTROL_SOURCE);
    if (pitchControlType == ANGLE_CONTROL) {
        setPitchAngleSetpoint(getPitchAngleInput(pitchControlSource));
        setPitchRateSetpoint(PIDcontrol(getPID(PITCH_ANGLE), getPitchAngleSetpoint() - getPitch(), MAX_PITCH_RATE / MAX_PITCH_ANGLE));
    } 
    else if (pitchControlType == RATE_CONTROL) {
        setPitchRateSetpoint(getPitchRateInput(pitchControlSource));
    }
    
    if (getControlValue(HEADING_CONTROL) == CONTROL_ON) { // if heading control is enabled
        setHeadingSetpoint(getHeadingInput(getControlValue(HEADING_CONTROL_SOURCE))); // get heading value (GS or AP)
        setYawRateSetpoint(PIDcontrol(getPID(HEADING), wrap_180(getHeadingSetpoint() - getHeading()), MAX_YAW_RATE / 180));
    } 
    else {
        setYawRateSetpoint(getYawRateInput(RC_SOURCE));
    }
        
    if (getControlValue(ALTITUDE_CONTROL) == CONTROL_ON) { // if altitude control is enabled
        setAltitudeSetpoint(getAltitudeInput(getControlValue(ALTITUDE_CONTROL_SOURCE))); // get altitude value (GS or AP)
        setThrottleSetpoint(PIDcontrol(getPID(ALTITUDE), getAltitudeSetpoint() - getAltitude(), 1) + getThrottleSetpoint());
    } 
    else { // if no altitude control, get raw throttle input (RC or GS)
        setThrottleSetpoint(getThrottleInput(getControlValue(THROTTLE_CONTROL_SOURCE)));
    }
}

void lowLevelControl() {  

    control_Roll = PIDcontrol(getPID(ROLL_RATE), getRollRateSetpoint() - getRollRate(), HALF_PWM_RANGE / MAX_ROLL_RATE);
    control_Pitch = PIDcontrol(getPID(PITCH_RATE), getPitchRateSetpoint() - getPitchRate(), HALF_PWM_RANGE / MAX_PITCH_RATE);
    control_Yaw = PIDcontrol(getPID(YAW_RATE), getYawRateSetpoint() - getYawRate(), HALF_PWM_RANGE / MAX_YAW_RATE);
    control_Throttle = getThrottleSetpoint();
    
    //Mixing!
    outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);

    //Error Checking
    checkLimits(outputSignal);

    if (control_Throttle > -850 && getProgramStatus() != KILL_MODE) {
        setAllPWM(outputSignal);
    } else {
        setPWM(1, MIN_PWM);
        setPWM(2, MIN_PWM);
        setPWM(3, MIN_PWM);
        setPWM(4, MIN_PWM);
    }
}

#endif