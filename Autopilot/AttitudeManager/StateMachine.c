/* 
 * File:   StateMachine.c
 * Author: Chris Hajduk
 *
 * Created on June 9, 2015, 9:00 PM
 */

#include "StateMachine.h"
#include "Probe_Drop.h"
#include "../Common/debug.h"
#include "../Common/Common.h"
#include "main.h"
#include "ProgramStatus.h"

/*
 * 
 */

//State Machine Triggers (Mostly Timers)
int dmaTimer = 0;
int uplinkTimer = 0;
int downlinkP0Timer = 0;
int downlinkP1Timer = 0;
int downlinkP2Timer = 0;
int imuTimer = 0;
char AMUpdate = 0;
long int stateMachineTimer = 0;
int dTime = 0;

//Important Autopilot Variables
int outputSignal[5];
int control_Roll, control_Pitch, control_Yaw, control_Throttle, control_Flap;
char killingPlane = 0;

void StateMachine(char entryLocation){
    //Timers
    dTime = (int)(getTime() - stateMachineTimer);
    stateMachineTimer = getTime();
    uplinkTimer += dTime;
    downlinkP0Timer += dTime;
    downlinkP1Timer += dTime;
    downlinkP2Timer += dTime;
    stateMachineTimer += dTime;
    imuTimer += dTime;
    dmaTimer += dTime;
    
    
    //Clear Watchdog timer
    asm("CLRWDT");
    //Feedback systems such as this autopilot are very sensitive to timing. In order to keep it consistent we should try to keep the timing between the calculation of error corrections and the output the same.
    //In other words, roll pitch and yaw control, mixing, and output should take place in the same step.
    if(AMUpdate){
        AMUpdate = 0;
        //Run - Angle control, and angular rate control
        //Input from Controller
        inputCapture();
        //Recalculate all data dependent on any DMA data
        highLevelControl();
        lowLevelControl();
    }
    else if(IMU_UPDATE_FREQUENCY <= imuTimer && entryLocation != STATEMACHINE_IMU){
//        debug("IMU");
        imuTimer = 0;
        //Poll Sensor
        imuCommunication();
        //Input from Controller
        inputCapture();
        highLevelControl();
        lowLevelControl();

    }
    else if(isDMADataAvailable() && checkDMA()){
        //Input from Controller
        inputCapture();
        //Recalculate all data dependent on any DMA data
        highLevelControl();
        lowLevelControl();
    }
    else{
    }

    if(UPLINK_CHECK_FREQUENCY <= uplinkTimer){
        uplinkTimer = 0;
        readDatalink();
    }

    if(P0_SEND_FREQUENCY <= downlinkP0Timer){
//        debug("P0");
        //Compile and send data
        downlinkP0Timer = 0;
        writeDatalink(PRIORITY0);
    }
    else if(P1_SEND_FREQUENCY <= downlinkP1Timer){
//        debug("P1");
        //Compile and send data
        downlinkP1Timer = 0;
        writeDatalink(PRIORITY1);
    }
    else if(P2_SEND_FREQUENCY <= downlinkP2Timer || areGainsUpdated()){
//        debug("P2");
        //Compile and send data
        downlinkP2Timer = 0;
        writeDatalink(PRIORITY2);
    }
    else{
        //Then Sleep
    }
    //Loop it back again!
    inboundBufferMaintenance();
    outboundBufferMaintenance();
    asm("CLRWDT");
}

#if FIXED_WING
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
        setPWM(PITCH_OUT_CHANNEL, outputSignal[1]); //Pitch
        setPWM(THROTTLE_OUT_CHANNEL, outputSignal[2]);//Throttle
        setPWM(YAW_OUT_CHANNEL, outputSignal[3]); //Yaw
    }
    else{
        setPWM(ROLL_OUT_CHANNEL, MIN_PWM);//Roll
        setPWM(PITCH_OUT_CHANNEL, MIN_PWM); //Pitch
        setPWM(THROTTLE_OUT_CHANNEL, MIN_PWM);//Throttle
        setPWM(YAW_OUT_CHANNEL, MIN_PWM); //Yaw
    }
    setPWM(FLAP_OUT_CHANNEL, outputSignal[4]); //Flaps
    setPWM(PROBE1_OUT_CHANNEL, probePWM(PROBE1));
    setPWM(PROBE2_OUT_CHANNEL, probePWM(PROBE2));
    setPWM(PROBE3_OUT_CHANNEL, probePWM(PROBE3));

    //Check for kill mode
#if COMP_MODE
    checkGPS();
    checkHeartbeat();
    checkUHFStatus();
#endif

}
#elif COPTER
void highLevelControl(){
    setYawRateSetpoint(headingControl(getHeadingSetpoint(), getHeading()));       //Keep a steady Roll Heading
}

void lowLevelControl(){
    control_Throttle = throttleControl(getAltitudeInput(), getAltitude());       //Hold a steady throttle (more or less airspeed for fixed wings)
    setRollRateSetpoint(rollAngleControl(getRollAngleSetpoint(), getRoll()));       //Keep a steady Roll Angle
    setPitchRateSetpoint(pitchAngleControl(getPitchAngleSetpoint(), getPitch()));   //Keep a steady Pitch Angle
    control_Roll = rollRateControl(getRollRateSetpoint(), getRollRate());
    control_Pitch = pitchRateControl(getPitchRateSetpoint(), getPitchRate());
    control_Yaw = yawRateControl(getYawRateSetpoint(), getYawRate());

    //Mixing!
    outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw, &control_Flap);

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

void killPlane(char action){
    if (action){
        killingPlane = 1;
        setProgramStatus(KILL_MODE);
    }
    else{
        killingPlane = 0;
        setProgramStatus(MAIN_EXECUTION);
    }
}

