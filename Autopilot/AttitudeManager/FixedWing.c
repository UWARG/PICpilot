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

char vehicleArmed = 0;

void initialization(int* outputSignal){
    setPWM(THROTTLE_OUT_CHANNEL, MIN_PWM);
    p_priority numPacket = PRIORITY1;
    setProgramStatus(UNARMED);
//    char str[20];
//    sprintf(str,"AM:%d, PM:%d",sizeof(AMData), sizeof(PMData));
//    debug(str);
    while (!vehicleArmed){
        imuCommunication();
        checkDMA();
        asm("CLRWDT");
        writeDatalink(numPacket%3);
        readDatalink();
        inboundBufferMaintenance();
        outboundBufferMaintenance();
        asm("CLRWDT");
        Delay(200);
        asm("CLRWDT");
        numPacket = (numPacket + 1) % 3;
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
    for (i = 1; i <= NUM_CHANNELS; i++){
        setPWM(i, MIN_PWM);
    }
    setProgramStatus(UNARMED);
    p_priority numPacket = PRIORITY1;
    while (!vehicleArmed){
        imuCommunication();
        asm("CLRWDT");
        writeDatalink(numPacket%3); //TODO: Change this for multiple packets
        readDatalink();
        inboundBufferMaintenance();
        outboundBufferMaintenance();
        Delay(200);
        asm("CLRWDT");
        numPacket = (numPacket + 1) % 3;
    }
    setProgramStatus(MAIN_EXECUTION);
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
    control_Flap = getFlapSetpoint();
    //Mixing!
    outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw, &control_Flap);
    //Error Checking
    checkLimits(outputSignal);
    //Then Output
    unsigned int cameraCounter = 0; //TODO: TEMPORARY, STORE TIME NOT COUNTER (OR BOTH) IMPLEMENT THIS A BETTER WAY
    cameraPollingRuntime(getLatitude(), getLongitude(), getTime(), &cameraCounter, getRoll(), getPitch());
    cameraGimbalStabilization(getRoll());
    goProGimbalStabilization(getRoll());
    goProVerticalstabilization(getPitch());
    //For fixed-wing aircraft: Typically 0 = Roll, 1 = Pitch, 2 = Throttle, 3 = Yaw

    if (!killingPlane){
        setPWM(ROLL_OUT_CHANNEL, outputSignal[0]);//Roll
        setPWM(L_TAIL_OUT_CHANNEL, outputSignal[1]); //Pitch
        setPWM(THROTTLE_OUT_CHANNEL, outputSignal[2]);//Throttle
        setPWM(R_TAIL_OUT_CHANNEL, outputSignal[3]); //Yaw
    }
    else{
        setPWM(ROLL_OUT_CHANNEL, MIN_PWM);//Roll
        setPWM(L_TAIL_OUT_CHANNEL, MIN_PWM); //Pitch
        setPWM(THROTTLE_OUT_CHANNEL, MIN_PWM);//Throttle
        setPWM(R_TAIL_OUT_CHANNEL, MIN_PWM); //Yaw
    }
    setPWM(FLAP_OUT_CHANNEL, outputSignal[4]); //Flaps

    //Check for kill mode
#if COMP_MODE
    checkGPS();
    checkHeartbeat();
    checkUHFStatus();
#endif

}

void takeOff(){

}
void landing(){
    
}

void inputMixing(int* channels, int* rollRate, int* pitchRate, int* throttle, int* yawRate, int* flap){
        if (getControlPermission(ROLL_CONTROL_SOURCE, ROLL_RC_SOURCE,ROLL_CONTROL_SOURCE_SHIFT)){
            (*rollRate) = channels[0];
        }
        if (getControlPermission(THROTTLE_CONTROL_SOURCE, THROTTLE_RC_SOURCE,THROTTLE_CONTROL_SOURCE_SHIFT))
            (*throttle) = (channels[2]);
        
        if (getControlPermission(FLAP_CONTROL_SOURCE, FLAP_RC_SOURCE,FLAP_CONTROL_SOURCE_SHIFT))
            (*flap) = channels[FLAP_IN_CHANNEL-1];

        #if TAIL_TYPE == STANDARD_TAIL
        if (getControlPermission(PITCH_CONTROL_SOURCE, PITCH_RC_SOURCE,0)){
            (*pitchRate) = channels[1];
        }
        (*yawRate) = channels[3];
        
        #elif TAIL_TYPE == V_TAIL    //V-tail
        // TODO

        #elif TAIL_TYPE == INV_V_TAIL
        if (getControlPermission(PITCH_CONTROL_SOURCE, PITCH_RC_SOURCE,0)){
            (*pitchRate) = (channels[1] - channels[3]) / (2 * ELEVATOR_PROPORTION);
        }
        (*yawRate) = (channels[1] + channels[3] ) / (2 * RUDDER_PROPORTION);
        
        #endif
}

void outputMixing(int* channels, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw, int* control_Flap){
    //code for different tail configurations
    #if TAIL_TYPE == STANDARD_TAIL  //is a normal t-tail
    channels[1] = (*control_Pitch);
    channels[3] = (*control_Yaw);

    #elif TAIL_TYPE == V_TAIL    //V-tail
    // TODO

    #elif TAIL_TYPE == INV_V_TAIL   //Inverse V-Tail
    channels[1] =  (*control_Yaw) * RUDDER_PROPORTION + (*control_Pitch) * ELEVATOR_PROPORTION ; //Tail Output Right
    channels[3] =  (*control_Yaw) * RUDDER_PROPORTION - (*control_Pitch) * ELEVATOR_PROPORTION ; //Tail Output Left
    
    #endif
    channels[0] = (*control_Roll);
    channels[2] = (*control_Throttle);
    channels[4] = (*control_Flap);
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
    if (channels[1] > MAX_L_TAIL_PWM) {
        channels[1] = MAX_L_TAIL_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
//        if (getIntegralSum(PITCH) * getGain(PITCH, GAIN_KI) > sp_PitchAngle - sp_ComputedPitchRate) {
//            setIntegralSum(PITCH, getIntegralSum(PITCH)/1.1);
//        }
    }
    else if (channels[1] < MIN_L_TAIL_PWM) {
        channels[1] = MIN_L_TAIL_PWM;
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

    if (channels[3] > MAX_R_TAIL_PWM)
        channels[3] = MAX_R_TAIL_PWM;
    else if (channels[3] < MIN_R_TAIL_PWM)
        channels[3] = MIN_R_TAIL_PWM;
    
    //Flaps
    if (channels[4] > MAX_PWM){
        channels[4] = MAX_PWM;
    }
}

void startArm()
{
    vehicleArmed = 1;
    armVehicle(500);
}

void stopArm()
{
    vehicleArmed = 0;
    dearmVehicle();
}
