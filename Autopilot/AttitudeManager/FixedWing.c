/**
 * @file   FixedWing.c
 * @author Chris Hajduk
 * @date July 2, 2015
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#include "PWM.h"
#include "AttitudeManager.h"
#include "FixedWing.h"
#include "ProgramStatus.h"
#include "StatusManager.h"
#include "../Common/Interfaces/InterchipDMA.h"
#include "../Common/Utilities/Logger.h"

#if VEHICLE_TYPE == FIXED_WING

#define AIRSPEED 0

static int outputSignal[NUM_CHANNELS];
static int control_Roll, control_Pitch, control_Yaw, control_Throttle;

int input_RC_Flap;
int input_GS_Flap;

float adverse_yaw_mix = 0.5; // Roll rate -> yaw rate scaling (to counter adverse yaw)
float roll_turn_mix = 1.0; // Roll angle -> pitch rate scaling (for banked turns) 

void initialization(){
    setPWM(THROTTLE_OUT_CHANNEL, MIN_PWM);

    int channel;
    for (channel = 0; channel < NUM_CHANNELS; channel++) {
        outputSignal[channel] = 0;
    }
    setProgramStatus(UNARMED);

    while (getProgramStatus() == UNARMED){
        StateMachine(STATEMACHINE_IDLE);
    }
}

void armVehicle(){
    setProgramStatus(ARMING);
    
    resetHeartbeatTimer();

    setPWM(THROTTLE_OUT_CHANNEL, MIN_PWM);
    setPWM(ROLL_OUT_CHANNEL, 0);
    setPWM(L_TAIL_OUT_CHANNEL, 0);
    setPWM(R_TAIL_OUT_CHANNEL, 0);
    setPWM(FLAP_OUT_CHANNEL, MIN_PWM);

}

void dearmVehicle(){
    int i;
    for (i = 1; i <= NUM_CHANNELS; i++){
        setPWM(i, MIN_PWM);
    }
    setProgramStatus(UNARMED);

    while (getProgramStatus() == UNARMED){
        StateMachine(STATEMACHINE_IDLE);
    }
    setProgramStatus(MAIN_EXECUTION);
}

void inputMixing(int* channelIn, int* rollRate, int* pitchRate, int* throttle, int* yawRate){
    if (getControlValue(THROTTLE_CONTROL_SOURCE) == RC_SOURCE) {
        *throttle = channelIn[THROTTLE_IN_CHANNEL - 1];
    }

#if TAIL_TYPE == STANDARD_TAIL
    if (getControlValue(ROLL_CONTROL_SOURCE) == RC_SOURCE){
        *rollRate = -channelIn[ROLL_IN_CHANNEL - 1];
    }
    if (getControlValue(PITCH_CONTROL_SOURCE) == RC_SOURCE){
        *pitchRate = -channelIn[PITCH_IN_CHANNEL - 1];
    }
    *yawRate = -channelIn[YAW_IN_CHANNEL - 1];

#elif TAIL_TYPE == V_TAIL
    
#elif TAIL_TYPE == INV_V_TAIL
    if (getControlValue(ROLL_CONTROL_SOURCE) == RC_SOURCE) {
        *rollRate = channelIn[ROLL_IN_CHANNEL - 1];
    }
    if (getControlValue(PITCH_CONTROL_SOURCE) == RC_SOURCE){
        *pitchRate = (channelIn[R_TAIL_IN_CHANNEL - 1] - channelIn[L_TAIL_IN_CHANNEL - 1]) / (2 * ELEVATOR_PROPORTION);
    }
    *yawRate = (channelIn[L_TAIL_IN_CHANNEL - 1] + channelIn[R_TAIL_IN_CHANNEL - 1] ) / (2 * RUDDER_PROPORTION);
#endif

    if (getControlValue(FLAP_CONTROL_SOURCE) == RC_SOURCE) {
        input_RC_Flap = channelIn[FLAP_IN_CHANNEL - 1];
    }

}
/*
 * Reference frames:
 * Uses NED frame (XYZ: North, East, Down)
 * In the plane's space this translates to:
 * +X = Forward
 * +Y = Right
 * +Z = Down
 * Angles are clockwise by those axes. Therefore:
 * +Roll = Right
 * +Pitch = Up
 * +Yaw = Right
 * These are the same as most IMUs will give. For consistency, do not modify the
 * orientation control with negatives. Add them where necessary to (controller)
 * input mixing, IMU reading, and output mixing. The standard input mixing is set 
 * up for standard controller layout (down+right is minimum values on sticks).
 */

void outputMixing(int* channelOut, int* control_Roll, int* control_Pitch, int* control_Throttle, int* control_Yaw){
    *control_Yaw += *control_Roll * adverse_yaw_mix; // mix roll rate into rudder to counter adverse yaw

    //code for different tail configurations
    #if TAIL_TYPE == STANDARD_TAIL  //is a normal t-tail
    channelOut[PITCH_OUT_CHANNEL - 1] = (*control_Pitch);
    channelOut[YAW_OUT_CHANNEL - 1] = (*control_Yaw);

    #elif TAIL_TYPE == V_TAIL    //V-tail
    // TODO

    #elif TAIL_TYPE == INV_V_TAIL   //Inverse V-Tail
    channelOut[L_TAIL_OUT_CHANNEL - 1] =  (*control_Yaw) * RUDDER_PROPORTION - (*control_Pitch) * ELEVATOR_PROPORTION ; //Tail Output Left
    channelOut[R_TAIL_OUT_CHANNEL - 1] =  (*control_Yaw) * RUDDER_PROPORTION + (*control_Pitch) * ELEVATOR_PROPORTION ; //Tail Output Right

    #endif
    channelOut[ROLL_OUT_CHANNEL - 1] = (*control_Roll);
    channelOut[THROTTLE_OUT_CHANNEL - 1] = (*control_Throttle);
}

void checkLimits(int* channelOut){
    constrain(&(channelOut[THROTTLE_OUT_CHANNEL - 1]), MIN_PWM, MAX_PWM);

    constrain(&(channelOut[ROLL_OUT_CHANNEL - 1]), MIN_ROLL_PWM, MAX_ROLL_PWM);

    constrain(&(channelOut[L_TAIL_OUT_CHANNEL - 1]), MIN_L_TAIL_PWM, MAX_L_TAIL_PWM);

    constrain(&(channelOut[R_TAIL_OUT_CHANNEL - 1]), MIN_R_TAIL_PWM, MAX_R_TAIL_PWM);

    constrain(&(channelOut[FLAP_OUT_CHANNEL - 1]), MIN_PWM, MAX_PWM);
}

void highLevelControl(){

    if (getControlValue(ALTITUDE_CONTROL) == CONTROL_ON) {
        setAltitudeSetpoint(getAltitudeInput(getControlValue(ALTITUDE_CONTROL_SOURCE)));
        setPitchAngleSetpoint(PIDcontrol(getPID(ALTITUDE), getAltitudeSetpoint() - getAltitude(), 1));
#if !AIRSPEED
        setThrottleSetpoint(PIDcontrol(getPID(ALTITUDE), getAltitudeSetpoint() - getAltitude(), HALF_PWM_RANGE / 2) + getThrottleSetpoint());
#endif
    } else {
        setPitchAngleSetpoint(getPitchAngleInput(getControlValue(PITCH_CONTROL_SOURCE)));
        setThrottleSetpoint(getThrottleInput(getControlValue(THROTTLE_CONTROL_SOURCE)));
    }

#if AIRSPEED // TODO: airspeed control. Need to verify sensor values.
    if (getControlValue(AIRSPEED_CONTROL) == CONTROL_ON) {
        setAirspeedSetpoint(gs_airspeed); // let the ground station always determine airspeed
        setThrottleSetpoint(PIDcontrol(getPID(AIRSPEED), getAirspeedSetpoint() - getAirspeed()), HALF_PWM_RANGE / 20);
        setFlaps(flaps_scale / getAirspeed()); // low speed -> more flaps
    }
#endif

    if (getControlValue(HEADING_CONTROL) == CONTROL_ON) {
        setHeadingSetpoint(getHeadingInput(getControlValue(HEADING_CONTROL_SOURCE)));
        setRollAngleSetpoint(PIDcontrol(getPID(HEADING), wrap_180(getHeadingSetpoint() - getHeading()), 1));
    } else {
        setRollAngleSetpoint(getRollAngleInput(getControlValue(ROLL_CONTROL_SOURCE)));
    }
}

void lowLevelControl(){
    if (getControlValue(ROLL_CONTROL_TYPE) == ANGLE_CONTROL || getControlValue(HEADING_CONTROL) == CONTROL_ON) {
        setRollRateSetpoint(PIDcontrol(getPID(ROLL_ANGLE), getRollAngleSetpoint() - getRoll(), MAX_ROLL_RATE / MAX_ROLL_ANGLE));
    } else {
        setRollRateSetpoint(getRollRateInput(getControlValue(ROLL_CONTROL_SOURCE)));
        setYawRateSetpoint(getYawRateInput(getControlValue(ROLL_CONTROL_SOURCE))); // No bit for yaw. If they have roll, they probably need yaw too.
    }

    if (getControlValue(PITCH_CONTROL_TYPE) == ANGLE_CONTROL || getControlValue(ALTITUDE_CONTROL) == CONTROL_ON){
        setPitchRateSetpoint(PIDcontrol(getPID(PITCH_ANGLE), getPitchAngleSetpoint() - getPitch(), MAX_PITCH_RATE / MAX_PITCH_ANGLE));
    } else {
        setPitchRateSetpoint(getPitchRateInput(getControlValue(PITCH_CONTROL_SOURCE)));
    }
    setPitchRateSetpoint(getPitchRateSetpoint() + (fabsf(getRoll()) * roll_turn_mix)); //Apply Coordinated Turn //Linear Function

    control_Roll = PIDcontrol(getPID(ROLL_RATE), getRollRateSetpoint() - getRollRate(), HALF_PWM_RANGE / MAX_ROLL_RATE);
    control_Pitch = PIDcontrol(getPID(PITCH_RATE), getPitchRateSetpoint() - getPitchRate(), HALF_PWM_RANGE / MAX_PITCH_RATE);
    control_Yaw = PIDcontrol(getPID(YAW_RATE), getYawRateSetpoint() - getYawRate(), HALF_PWM_RANGE / MAX_YAW_RATE);
    control_Throttle = getThrottleSetpoint();

    outputSignal[FLAP_OUT_CHANNEL - 1] = getFlapInput(getControlValue(FLAP_CONTROL_SOURCE)); // don't need to mix the flaps

    //Mixing!
    outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);

    //Error Checking
    checkLimits(outputSignal);
    //Then Output

    if (getProgramStatus() != KILL_MODE) {
        setAllPWM(outputSignal);
    } else{ //if in kill mode, full deflection of all control surfaces
        setPWM(THROTTLE_OUT_CHANNEL, MIN_PWM);  //Throttle
        setPWM(ROLL_OUT_CHANNEL, MIN_PWM);      //Roll
        setPWM(L_TAIL_OUT_CHANNEL, MIN_PWM);    //Pitch
        setPWM(R_TAIL_OUT_CHANNEL, MIN_PWM);    //Yaw
    }

    //Check for kill mode
#if COMP_MODE
    if (getUHFStatus() == CONNECTION_WARN && getProgramStatus() != KILL_MODE_WARNING && getProgramStatus() != KILL_MODE){
        info("Setting kill mode warning due to UHF disconnect");
        setProgramStatus(KILL_MODE_WARNING);
    } else if (getUHFStatus() == CONNECTION_EXPIRED && getProgramStatus() != KILL_MODE){
        info("Setting  kill mode due to UHF disconnect");
        setProgramStatus(KILL_MODE);
    }

    if (getHeartbeatStatus() == CONNECTION_WARN && getProgramStatus() != KILL_MODE_WARNING && getProgramStatus() != KILL_MODE){
        interchip_send_buffer.am_data.command = PM_RETURN_HOME;
        sendInterchipData();
        info("Setting kill mode warning due to HEARTBEAT");
        setProgramStatus(KILL_MODE_WARNING);
    } else if (getHeartbeatStatus() == CONNECTION_EXPIRED && getProgramStatus() != KILL_MODE){
        info("Setting  kill mode due to HEARTBEAT");
        setProgramStatus(KILL_MODE);
    }
#endif

}

#endif
