/*
 * File:   FixedWing.c
 * Author: Chris Hajduk
 *
 * Created on July 2, 2015, 8:04 PM
 */

#include "PWM.h"
#include "AttitudeManager.h"
#include "delay.h"
#include "FixedWing.h"
#include "ProgramStatus.h"

#if VEHICLE_TYPE == FIXED_WING

int outputSignal[NUM_CHANNELS];
int control_Roll, control_Pitch, control_Yaw, control_Throttle;

void initialization(){
    setPWM(THROTTLE_OUT_CHANNEL, MIN_PWM);
    
    int channel = 0;
    for (; channel < NUM_CHANNELS; channel++) {
        outputSignal[channel] = 0;
    }
    setProgramStatus(UNARMED);

//    char str[20];
//    sprintf(str,"AM:%d, PM:%d",sizeof(AMData), sizeof(PMData));
//    debug(str);
    while (getProgramStatus() == UNARMED){
        StateMachine(STATEMACHINE_IDLE);
    }
}

void armVehicle(int delayTime){
    setProgramStatus(ARMING);
    asm("CLRWDT");
    Delay(delayTime);
    asm("CLRWDT");
    setPWM(THROTTLE_OUT_CHANNEL, MIN_PWM);
    setPWM(ROLL_OUT_CHANNEL, 0);
    setPWM(L_TAIL_OUT_CHANNEL, 0);
    setPWM(R_TAIL_OUT_CHANNEL, 0);
    setPWM(FLAP_OUT_CHANNEL, MIN_PWM);
    asm("CLRWDT");
    Delay(delayTime);
    asm("CLRWDT");
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

void inputMixing(int* channelIn, int* rollRate, int* pitchRate, int* throttle, int* yawRate){
        if (getControlPermission(ROLL_CONTROL_SOURCE, ROLL_RC_SOURCE,ROLL_CONTROL_SOURCE_SHIFT)){
            (*rollRate) = channelIn[ROLL_IN_CHANNEL - 1];
        }
        if (getControlPermission(THROTTLE_CONTROL_SOURCE, THROTTLE_RC_SOURCE,THROTTLE_CONTROL_SOURCE_SHIFT)) {
            (*throttle) = (channelIn[THROTTLE_IN_CHANNEL - 1]);
        }

        #if TAIL_TYPE == STANDARD_TAIL
        if (getControlPermission(PITCH_CONTROL_SOURCE, PITCH_RC_SOURCE,0)){
            (*pitchRate) = channelIn[PITCH_IN_CHANNEL - 1];
        }
        (*yawRate) = channelIn[YAW_IN_CHANNEL - 1];
        
        #elif TAIL_TYPE == V_TAIL    //V-tail
        // TODO

        #elif TAIL_TYPE == INV_V_TAIL
        if (getControlPermission(PITCH_CONTROL_SOURCE, PITCH_RC_SOURCE,0)){
            (*pitchRate) = (channelIn[L_TAIL_IN_CHANNEL - 1] - channelIn[R_TAIL_IN_CHANNEL - 1]) / (2 * ELEVATOR_PROPORTION);
        }
        (*yawRate) = (channelIn[L_TAIL_IN_CHANNEL - 1] + channelIn[R_TAIL_IN_CHANNEL - 1] ) / (2 * RUDDER_PROPORTION);
        
        #endif
}

void outputMixing(int* channelOut, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw){
    //code for different tail configurations
    #if TAIL_TYPE == STANDARD_TAIL  //is a normal t-tail
    channelsOut[PITCH_OUT_CHANNEL] = (*control_Pitch);
    channelsOut[YAW_OUT_CHANNEL] = (*control_Yaw);

    #elif TAIL_TYPE == V_TAIL    //V-tail
    // TODO

    #elif TAIL_TYPE == INV_V_TAIL   //Inverse V-Tail
    channelOut[R_TAIL_OUT_CHANNEL - 1] =  (*control_Yaw) * RUDDER_PROPORTION + (*control_Pitch) * ELEVATOR_PROPORTION ; //Tail Output Right
    channelOut[L_TAIL_OUT_CHANNEL - 1] =  (*control_Yaw) * RUDDER_PROPORTION - (*control_Pitch) * ELEVATOR_PROPORTION ; //Tail Output Left
    
    #endif
    channelOut[ROLL_OUT_CHANNEL - 1] = (*control_Roll);
    channelOut[THROTTLE_OUT_CHANNEL - 1] = (*control_Throttle);
}

void checkLimits(int* channelOut){
    if (channelOut[ROLL_OUT_CHANNEL - 1] > MAX_ROLL_PWM) {
        channelOut[ROLL_OUT_CHANNEL - 1] = MAX_ROLL_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
//        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) > sp_RollRate - sp_ComputedRollRate) {
//            setIntegralSum(ROLL, getIntegralSum(ROLL)/1.1);
//        }
    }
    if (channelOut[ROLL_OUT_CHANNEL - 1] < MIN_ROLL_PWM) {
        channelOut[ROLL_OUT_CHANNEL - 1] = MIN_ROLL_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
//        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) < sp_RollAngle - sp_ComputedRollRate) {
//            setIntegralSum(ROLL, getIntegralSum(ROLL)/1.1);
//        }
    }
    if (channelOut[L_TAIL_OUT_CHANNEL - 1] > MAX_L_TAIL_PWM) {
        channelOut[L_TAIL_OUT_CHANNEL - 1] = MAX_L_TAIL_PWM;
    } else if (channelOut[L_TAIL_OUT_CHANNEL - 1] < MIN_L_TAIL_PWM) {
        channelOut[L_TAIL_OUT_CHANNEL - 1] = MIN_L_TAIL_PWM;
    }
    
    //Throttle = 1
    if (channelOut[THROTTLE_OUT_CHANNEL - 1] > MAX_PWM) {
        channelOut[THROTTLE_OUT_CHANNEL - 1] = MAX_PWM;
    } else if (channelOut[THROTTLE_OUT_CHANNEL - 1] < MIN_PWM){
        channelOut[THROTTLE_OUT_CHANNEL - 1] = MIN_PWM;
    }

    if (channelOut[R_TAIL_OUT_CHANNEL - 1] > MAX_R_TAIL_PWM) {
        channelOut[R_TAIL_OUT_CHANNEL - 1] = MAX_R_TAIL_PWM;
    } else if (channelOut[R_TAIL_OUT_CHANNEL - 1] < MIN_R_TAIL_PWM) {
        channelOut[R_TAIL_OUT_CHANNEL - 1] = MIN_R_TAIL_PWM;
    }
    
    //Flaps
    if (channelOut[FLAP_OUT_CHANNEL - 1] > MAX_PWM){
        channelOut[FLAP_OUT_CHANNEL - 1] = MAX_PWM;
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

void lowLevelControl(){
    //If commands come from the autopilot
    if (getControlPermission(ROLL_CONTROL_TYPE, ANGLE_CONTROL,ROLL_CONTROL_TYPE_SHIFT) || getControlPermission(HEADING_CONTROL,HEADING_CONTROL_ON, HEADING_CONTROL_SHIFT)) setRollRateSetpoint(rollAngleControl(getRollAngleSetpoint(), -getRoll()));       //Keep a steady Roll Angle
    //If commands come from the ground station
    else if (getControlPermission(ROLL_CONTROL_SOURCE, ROLL_GS_SOURCE,ROLL_CONTROL_SOURCE_SHIFT) || getControlPermission(ALTITUDE_CONTROL,ALTITUDE_CONTROL_ON,0)) setRollRateSetpoint(getRollRateInput(ROLL_GS_SOURCE));
    //If commands come from the RC Controller
    else setRollRateSetpoint(getRollRateInput(ROLL_RC_SOURCE));

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
    if (getControlPermission(FLAP_CONTROL_SOURCE, FLAP_GS_SOURCE ,FLAP_CONTROL_SOURCE_SHIFT)){setFlapSetpoint(getFlapInput(FLAP_GS_SOURCE));}
    //If commands come from the RC Controller
    else if (getControlPermission(FLAP_CONTROL_SOURCE,FLAP_RC_SOURCE, FLAP_CONTROL_SOURCE_SHIFT)){setFlapSetpoint(getFlapInput(FLAP_RC_SOURCE));}

    //If commands come from the ground station
    if (getControlPermission(THROTTLE_CONTROL_SOURCE, THROTTLE_GS_SOURCE ,THROTTLE_CONTROL_SOURCE_SHIFT)){setThrottleSetpoint(getThrottleInput(THROTTLE_GS_SOURCE));}
    //If commands come from the RC Controller
    else if (getControlPermission(THROTTLE_CONTROL_SOURCE,THROTTLE_RC_SOURCE, THROTTLE_CONTROL_SOURCE_SHIFT)){setThrottleSetpoint(getThrottleInput(THROTTLE_RC_SOURCE));}

    control_Roll = rollRateControl((float)getRollRateSetpoint(), -getRollRate());
    control_Pitch = pitchRateControl((float)getPitchRateSetpoint(), -getPitchRate());
    control_Yaw = yawRateControl((float)getYawRateSetpoint(), -getYawRate());
    control_Throttle = getThrottleSetpoint();
    
    outputSignal[FLAP_OUT_CHANNEL - 1] = getFlapSetpoint(); // don't need to mix the flaps

    //Mixing!
    outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);
    
    //Error Checking
    checkLimits(outputSignal);
    //Then Output
    unsigned int cameraCounter = 0; //TODO: TEMPORARY, STORE TIME NOT COUNTER (OR BOTH) IMPLEMENT THIS A BETTER WAY
    cameraPollingRuntime(getLatitude(), getLongitude(), getTime(), &cameraCounter, getRoll(), getPitch());
    cameraGimbalStabilization(getRoll());
    goProGimbalStabilization(getRoll());
    goProVerticalstabilization(getPitch());
    //For fixed-wing aircraft: Typically 0 = Roll, 1 = Pitch, 2 = Throttle, 3 = Yaw

    if (getProgramStatus() != KILL_MODE) {
        setAllPWM(outputSignal); //Yaw
    }
    else{
        setPWM(ROLL_OUT_CHANNEL, MIN_PWM);//Roll
        setPWM(L_TAIL_OUT_CHANNEL, MIN_PWM); //Pitch
        setPWM(THROTTLE_OUT_CHANNEL, MIN_PWM);//Throttle
        setPWM(R_TAIL_OUT_CHANNEL, MIN_PWM); //Yaw
    }

    //Check for kill mode
#if COMP_MODE
    checkGPS();
    checkHeartbeat();
    checkUHFStatus();
#endif

}

#endif