/* 
 * File:   StateMachine.c
 * Author: Chris Hajduk
 *
 * Created on June 9, 2015, 9:00 PM
 */

#include "StateMachine.h"
/*
 * 
 */


//State Machine Triggers (Mostly Timers)
int uplinkTimer = 0;
int downlinkTimer = 0;
int imuTimer = 0;
char AMUpdate = 0;
long int stateMachineTimer = 0;
int dTime = 0;

//Important Autopilot Variables
int outputSignal[4];
int control_Roll, control_Pitch, control_Yaw, control_Throttle;

void StateMachine(){
    //Timers
    dTime = (int)(getTime() - stateMachineTimer);
    stateMachineTimer = getTime();
    uplinkTimer += dTime;
    downlinkTimer += dTime;
    stateMachineTimer += dTime;
    imuTimer += dTime;
    if(isDMADataAvailable() && checkDMA()){
        debug("DMA");
//        //Input from Controller
//        inputCapture();
//        //Recalculate all data dependent on any DMA data
//        highLevelControl();
//        lowLevelControl();
    }
    //Feedback systems such as this autopilot are very sensitive to timing. In order to keep it consistent we should try to keep the timing between the calculation of error corrections and the output the same.
    //In other words, roll pitch and yaw control, mixing, and output should take place in the same step.
//    else if(AMUpdate){
//        debug("FORCE UPDATE");
//        AMUpdate = 0;
//        //Run - Angle control, and angular rate control
//        //Input from Controller
//        inputCapture();
//        //Recalculate all data dependent on any DMA data
//        highLevelControl();
//        lowLevelControl();
//    }
    else if(IMU_UPDATE_FREQUENCY <= imuTimer){
        debug("IMU");
        imuTimer = 0;
        //Poll Sensor
        imuCommunication();
        //Input from Controller
//        inputCapture();
//        lowLevelControl();

    }
    else if(DATALINK_SEND_FREQUENCY <= downlinkTimer){
        //Compile and send data
        debug("DOWNLINK");
        downlinkTimer = 0;
        writeDatalink();
        outboundBufferMaintenance();
    }
//    else if(UPLINK_CHECK_FREQUENCY <= uplinkTimer){
//        debug("UPLINK");
//        uplinkTimer = 0;
//        readDatalink();
//        inboundBufferMaintenance();
//    }
//    else{
//
//        //Then Sleep
//    }
    //Loop it back again!
    asm("CLRWDT");
}

#if FIXED_WING
void highLevelControl(){
    setPitchAngleSetpoint(altitudeControl(getAltitudeSetpoint(), getAltitude()));   //Hold a steady altitude
    control_Throttle = throttleControl(getAltitudeSetpoint(), getAltitude());       //Hold a steady throttle (more or less airspeed for fixed wings)
    setRollAngleSetpoint(headingControl(getHeadingSetpoint(), getHeading()));       //Keep a steady Roll Heading
}

void lowLevelControl(){
    setRollRateSetpoint(rollAngleControl(getRollAngleSetpoint(), getRoll()));       //Keep a steady Roll Angle
    setPitchRateSetpoint(pitchAngleControl(getPitchAngleSetpoint(), getPitch()));   //Keep a steady Pitch Angle
    setPitchRateSetpoint(coordinatedTurn(getPitchRateSetpoint(), getRoll()));       //Apply Coordinated Turn
    control_Roll = rollRateControl(getRollRateSetpoint(), getRollRate());
    control_Pitch = pitchRateControl(getPitchRateSetpoint(), getPitchRate());
    control_Yaw = yawRateControl(getYawRateSetpoint(), getYawRate());
    //Mixing!
    outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);
    //Error Checking
    checkLimits(outputSignal);
    //Then Output
    unsigned int cameraCounter = 0; //TODO: TEMPORARY, STORE TIME NOT COUNTER (OR BOTH) IMPLEMENT THIS A BETTER WAY
    unsigned int cameraPWM = cameraPollingRuntime(getLatitude(), getLongitude(), getTime(), &cameraCounter, getRoll(), getPitch());
    unsigned int gimbalPWM = cameraGimbalStabilization(getRoll());
    unsigned int goProGimbalPWM = goProGimbalStabilization(getRoll());
    unsigned int verticalGoProPWM = goProVerticalstabilization(getPitch());
    //For fixed-wing aircraft: Typically 0 = Roll, 1 = Pitch, 2 = Throttle, 3 = Yaw
    setPWM(1, outputSignal[0]);//Roll
    setPWM(2, outputSignal[1]); //Pitch
    setPWM(3, outputSignal[2]);//Throttle
    setPWM(4, outputSignal[3]); //Yaw
    setPWM(5, goProGimbalPWM);
    setPWM(6, verticalGoProPWM);
    setPWM(7, gimbalPWM);
    setPWM(8, cameraPWM);
}
#elif COPTER //TODO TOMORROW FINISH/CHANGE
void highLevelControl(){
    control_Throttle = throttleControl(getAltitudeSetpoint(), getAltitude());       //Hold a steady throttle (more or less airspeed for fixed wings)
    setYawRateSetpoint(headingControl(getHeadingSetpoint(), getHeading()));       //Keep a steady Roll Heading
}

void lowLevelControl(){
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
    setPWM(1, outputSignal[0]);//Roll
    setPWM(2, outputSignal[1]); //Pitch
    setPWM(3, outputSignal[2]);//Throttle
    setPWM(4, outputSignal[3]); //Yaw
}
#endif
void forceStateMachineUpdate(){
    AMUpdate = 1;
}

