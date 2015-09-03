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

// Here there should be IOMATRIX_SIZE macros defined - One for each row
#define GET_MATRIX_ROW0 getMatrixValue(IOMatrix[0],valueMatrix[0])
#define GET_MATRIX_ROW1 getMatrixValue(IOMatrix[1],valueMatrix[1])
#define GET_MATRIX_ROW2 getMatrixValue(IOMatrix[2],valueMatrix[2])
#define GET_MATRIX_ROW3 getMatrixValue(IOMatrix[3],valueMatrix[3])
#define GET_MATRIX_ROW4 getMatrixValue(IOMatrix[4],valueMatrix[4])
#define GET_MATRIX_ROW5 getMatrixValue(IOMatrix[5],valueMatrix[5])
#define GET_MATRIX_ROW6 getMatrixValue(IOMatrix[6],valueMatrix[6])
#define GET_MATRIX_ROW7 getMatrixValue(IOMatrix[7],valueMatrix[7])
#define GET_MATRIX_ROW8 getMatrixValue(IOMatrix[8],valueMatrix[8])

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
float IOMatrix[][IOMATRIX_SIZE] = { //Add multiplication factors here
{0, 0, 0, 0, 0, 0, 0, 0, 0},
{1, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 1, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 1, 0, 0, 0, 0},
{0, 0, 1, 0, 0, 0, 0, 0, 0},
};
//Stores the inputs and outputs to each function (see above)
int valueMatrix[IOMATRIX_SIZE][IOMATRIX_SIZE];

//State Machine Triggers (Mostly Timers)
int uplinkTimer = 0;
int downlinkTimer = 0;
//int bufferTimer = 0;
int imuTimer = 0;
char AMUpdate = 0;
long int stateMachineTimer = 0;
int dTime = 0;

//Important Autopilot Variables
int outputSignal[4];
int control_Roll, control_Pitch, control_Yaw, control_Throttle;

void StateMachine(char* cond){
    //Timers
    dTime = (int)(getTime() - stateMachineTimer);
    uplinkTimer += dTime;
    downlinkTimer += dTime;
    stateMachineTimer += dTime;
    //State machine
    while(cond){
        if(isDMADataAvailable()){
            //Complete DMA Check
            checkDMA();
            //Recalculate all data dependent on any DMA data
            altitudeControl(GET_MATRIX_ROW0, getAltitude());//1st IO
            control_Throttle = throttleControl(GET_MATRIX_ROW1, getAltitude());
            headingControl(GET_MATRIX_ROW2, getHeading());
            rollAngleControl(GET_MATRIX_ROW3, getRoll());
            pitchAngleControl(GET_MATRIX_ROW4, getPitch());
            coordinatedTurn(GET_MATRIX_ROW5, getRoll());
            control_Roll = rollRateControl(GET_MATRIX_ROW6, getRollRate());
            control_Pitch = pitchRateControl(GET_MATRIX_ROW7, getPitchRate());
            control_Yaw = yawRateControl(GET_MATRIX_ROW8, getYawRate()); //9th IO
            //Mixing!
            outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);
            //Error Checking
            checkLimits(outputSignal);
            //Then Output
            unsigned int cameraCounter = 0; //TEMPORARY, STORE TIME NOT COUNTER (OR BOTH) IMPLEMENT THIS A BETTER WAY
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
        else if(DATALINK_SEND_FREQUENCY >= downlinkTimer){
            //Compile and send data
            downlinkTimer = 0;
            writeDatalink();
        }
        else if(UPLINK_CHECK_FREQUENCY >= uplinkTimer){
            uplinkTimer = 0;
            readDatalink();
        }
        //Feedback systems such as this autopilot are very sensitive to timing. In order to keep it consistent we should try to keep the timing between the calculation of error corrections and the output the same.
        //In other words, roll pitch and yaw control, mixing, and output should take place in the same step.
        else if(AMUpdate){
            AMUpdate = 0;
            //Run - Angle control, and angular rate control
            altitudeControl(GET_MATRIX_ROW0, getAltitude());//1st IO
            control_Throttle = throttleControl(GET_MATRIX_ROW1, getAltitude());
            headingControl(GET_MATRIX_ROW2, getHeading());
            rollAngleControl(GET_MATRIX_ROW3, getRoll());
            pitchAngleControl(GET_MATRIX_ROW4, getPitch());
            coordinatedTurn(GET_MATRIX_ROW5, getRoll());
            control_Roll = rollRateControl(GET_MATRIX_ROW6, getRollRate());
            control_Pitch = pitchRateControl(GET_MATRIX_ROW7, getPitchRate());
            control_Yaw = yawRateControl(GET_MATRIX_ROW8, getYawRate()); //9th IO
            //Mixing!
            outputMixing(outputSignal, &control_Roll, &control_Pitch, &control_Throttle, &control_Yaw);
            //Error Checking
            checkLimits(outputSignal);
            //Then Output
            unsigned int cameraCounter = 0; //TEMPORARY, STORE TIME NOT COUNTER (OR BOTH) IMPLEMENT THIS A BETTER WAY
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
        else if(IMU_UPDATE_FREQUENCY > imuTimer){
            imuTimer = 0;
            //Poll Sensor
            imuCommunication();
        }
//        else if(BUFFER_UPDATE_FREQUENCY > bufferTimer){
//
//        }
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

void forceStateMachineUpdate(){
    AMUpdate = 1;
}

