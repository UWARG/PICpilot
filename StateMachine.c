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

//Variables passed throughout the state machine are multiplied by this matrix
//They represent (in order):
//Altitude Setpoint
//Throttle Setpoint
//Heading Setpoint
//Roll Angle Setpoint
//Pitch Angle Setpoint
//Coordinated Turn Setpoint (Usually Roll Angle input)
//Roll Rate Setpoint
//Pitch Rate Setpoint
//Yaw Rate Setpoint
float IOMatrix[][IOMATRIX_SIZE] = {
{1, 2, 3, 4, 5, 6, 7, 8, 9},
{1, 2, 3, 4, 5, 6, 7, 8, 9},
{1, 2, 3, 4, 5, 6, 7, 8, 9},
{1, 2, 3, 4, 5, 6, 7, 8, 9},
{1, 2, 3, 4, 5, 6, 7, 8, 9},
{1, 2, 3, 4, 5, 6, 7, 8, 9},
{1, 2, 3, 4, 5, 6, 7, 8, 9},
{1, 2, 3, 4, 5, 6, 7, 8, 9},
{1, 2, 3, 4, 5, 6, 7, 8, 9},
};
//Stores the inputs and outputs to each function (see above)
int valueMatrix[IOMATRIX_SIZE][IOMATRIX_SIZE];

void StateMachine(char* cond){

    while(cond){
        if(DMAInterrupt){
            //Complete DMA Check
            checkDMA();
            //Recalculate all data dependent on any DMA data
            altitudeControl(getMatrixValue(&(IOMatrix[0]),&(valueMatrix[0])),);//1st IO
            throttleControl();
            headingControl();
            rollAngleControl();
            pitchAngleControl();
            coordinatedTurn();
            rollRateControl();
            pitchRateControl();
            yawRateControl(); //9th IO
            //Mixing!
            outputMixing(&outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);
            //Error Checking
            checkLimits(&outputSignal);
            //Then Output
            unsigned int cameraPWM = cameraPollingRuntime(gps_Latitude, gps_Longitude, time, &cameraCounter, imu_RollAngle, imu_PitchAngle);
            unsigned int gimbalPWM = cameraGimbalStabilization(imu_RollAngle);
            unsigned int goProGimbalPWM = goProGimbalStabilization(imu_RollAngle);
            unsigned int verticalGoProPWM = goProVerticalstabilization(imu_PitchAngle);
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
        else if(UplinkUupdate){
            readDatalink();
        }
        else if(DataLinkUpdate){
            //Compile and send data
            //Gives DataLinkParam, a parameter for deciding which of the next 2
            //if statements we run
            writeDatalink(DATALINK_SEND_FREQUENCY);
        }
        else if(doneAbove & DataLinkParam){
            //Poll Sensor
        }
        //Feedback systems such as this autopilot are very sensitive to timing. In order to keep it consistent we should try to keep the timing between the calculation of error corrections and the output the same.
        //In other words, roll pitch and yaw control, mixing, and output should take place in the same step.
        else if(!DataLinkParam & !doneAttitudeManager){
            //Run - Angle control, and angular rate control
            headingControl();
            rollAngleControl();
            pitchAngleControl();
            coordinatedTurn();
            rollRateControl();
            pitchRateControl();
            yawRateControl();
            //Mixing!
            outputMixing(&outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);
            //Error Checking
            checkLimits(&outputSignal);
            //Then Output
            unsigned int cameraPWM = cameraPollingRuntime(gps_Latitude, gps_Longitude, time, &cameraCounter, imu_RollAngle, imu_PitchAngle);
            unsigned int gimbalPWM = cameraGimbalStabilization(imu_RollAngle);
            unsigned int goProGimbalPWM = goProGimbalStabilization(imu_RollAngle);
            unsigned int verticalGoProPWM = goProVerticalstabilization(imu_PitchAngle);
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
        else{
            //Sleep
        }
        //Loop it back again!
    }
}
int getMatrixValue(float* IOMatrix, int* valueMatrix){
    int i = 0;
    int sum = 0;
    for (i = 0; i < IOMATRIX_SIZE; i++){
        sum += IOMatrix[i] * valueMatrix[i];
    }
    return sum;
}

