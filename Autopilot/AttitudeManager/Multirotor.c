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

char vehicleArmed = 0;

void initialization(){
    setPWM(FRONT_LEFT_MOTOR, MIN_PWM);
    setPWM(FRONT_RIGHT_MOTOR, MIN_PWM);
    setPWM(BACK_RIGHT_MOTOR, MIN_PWM);
    setPWM(BACK_LEFT_MOTOR, MIN_PWM);
    
    p_priority numPacket = PRIORITY1;
    setProgramStatus(UNARMED);
    
    while (!vehicleArmed){
        imuCommunication();
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

void lowLevelControl() {
    control_Throttle = throttleControl(getAltitudeInput(ALTITUDE_AP_SOURCE), getAltitude());       //Hold a steady throttle (more or less airspeed for fixed wings)
    setRollRateSetpoint(rollAngleControl(getRollAngleSetpoint(), getRoll()));       //Keep a steady Roll Angle
    setPitchRateSetpoint(pitchAngleControl(getPitchAngleSetpoint(), getPitch()));   //Keep a steady Pitch Angle
    control_Roll = rollRateControl(getRollRateSetpoint(), getRollRate());
    control_Pitch = pitchRateControl(getPitchRateSetpoint(), getPitchRate());
    control_Yaw = yawRateControl(getYawRateSetpoint(), getYawRate());

    //Mixing!
    outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);

    //Error Checking
    checkLimits(outputSignal);
    //For fixed-wing aircraft: Typically 0 = Roll, 1 = Pitch, 2 = Throttle, 3 = Yaw
    setPWM(FRONT_LEFT_MOTOR, outputSignal[0]);
    setPWM(FRONT_RIGHT_MOTOR, outputSignal[1]);
    setPWM(BACK_RIGHT_MOTOR, outputSignal[2]);
    setPWM(BACK_LEFT_MOTOR, outputSignal[3]); 
}

void takeOff(){

}
void landing(){

}

void inputMixing(int* channels, int* rollRate, int* pitchRate, int* throttle, int* yawRate, int* flap) { //no flaps on a quad
        if (getControlPermission(ROLL_CONTROL_SOURCE, ROLL_RC_SOURCE, ROLL_CONTROL_SOURCE_SHIFT)){
            (*rollRate) = channels[0];
        }
        if (getControlPermission(THROTTLE_CONTROL_SOURCE, THROTTLE_RC_SOURCE, THROTTLE_CONTROL_SOURCE_SHIFT)){
            (*throttle) = (channels[2]);
        }
        
        if (getControlPermission(PITCH_CONTROL_SOURCE, PITCH_RC_SOURCE, PITCH_CONTROL_SOURCE_SHIFT)){
            (*pitchRate) = channels[1];
        }
        
        (*yawRate) = channels[3];
}

void outputMixing(int* channels, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw){
    
    channels[0] = (*control_Throttle) + (*control_Pitch) - (*control_Roll) - (*control_Yaw);// + MIN_PWM;  //Front
    channels[1] = (*control_Throttle) + (*control_Pitch) + (*control_Roll) + (*control_Yaw);// + MIN_PWM;   //Left
    channels[2] = (*control_Throttle) - (*control_Pitch) + (*control_Roll) - (*control_Yaw);// + MIN_PWM;  //Back
    channels[3] = (*control_Throttle) - (*control_Pitch) - (*control_Roll) + (*control_Yaw);// + MIN_PWM;   //Right
}

void checkLimits(int* channels){
    for (char i = 0; i < 4; i++) {
        if (channels[i] > MAX_PWM) {
            channels[i] = MAX_PWM;
        } else if (channels[i] < MIN_PWM) {
            channels[i] = MIN_PWM;
        }
    }
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
