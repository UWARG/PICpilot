/* 
 * File:   IMU_Test_Main.c
 * Author: Mitch
 *
 * Created on June 15, 2013, 3:40 PM
 */

//Include Libraries
#include <stdio.h>
#include <stdlib.h>
#include "main.h"


//Include the Full Initialization Header File
#include "Clock.h"
#include "FullInitialize.h"
#include "delay.h"
#include "VN100.h"
#include "InputCapture.h"
#include "OutputCompare.h"
#include "net.h"
#include "StartupErrorCodes.h"

//Perform random clock things that came with the Sample Code (Keep in code)
_FOSCSEL(FNOSC_FRC); // Internal FRC oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);
// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystanl


#define YAW     0
#define PITCH   1
#define ROLL    2




/*****************************************************************************
 *****************************************************************************

                            STABILIZATION CODE

 *****************************************************************************
 *****************************************************************************/

//Servo scale factor is used in converting deg/s rotation input into output compare values
float SERVO_SCALE_FACTOR;
float integralSum = 0;

//float Angle_Bias[3];
int controlSignalPosition(float setpoint, float output, float gain, float integralGain, float dTime) { // function to find output based on gyro acceleration and PWM input
    integralSum += (setpoint - output) * dTime;
    int control = ((setpoint - output) * gain) + (integralSum * integralGain);
    return control;
}
int controlSignalAngles(float setpoint, float output, float gain, float SERVO_SCALE_FACTOR_ANGLES) { // function to find output based on gyro acceleration and PWM input
    int control = SERVO_SCALE_FACTOR_ANGLES * ((setpoint - output) * gain);
    return control;
}
int controlSignal(float setpoint, float output, float gain) { // function to find output based on gyro acceleration and PWM input
    int control = SERVO_SCALE_FACTOR * (setpoint - output * gain) + MIDDLE_PWM;
    return control;
}

//int getAngleBias(){
//    VN100_SPI_GetYPR(0, &Angle_Bias[YAW], &Angle_Bias[PITCH], &Angle_Bias[ROLL]);
//    }

/*****************************************************************************
 *****************************************************************************
 ******************************************************************************/

int main() {



    //Debug Mode:
    if (DEBUG) {
        InitUART1();  
    }
    checkErrorCodes();

    initDataLink();



    SERVO_SCALE_FACTOR = -(UPPER_PWM - MIDDLE_PWM) / 45;

    // Setpoints (From radio transmitter or autopilot)

    int sp_PitchRate = 0;
    int sp_ThrottleRate = 0;
    int sp_YawRate = 0;
    int sp_RollRate = 0;
    int sp_Range = 0; // +- Range for the sp_xxxxRate values

    int sp_Value = 0; //0=Roll, 1= Pitch, 2=Yaw
    int sp_Type = 0; //0 = Saved Value, 1 = Edit Mode
    int sp_Switch = 0;
    int sp_GearSwitch = 0;

    float sp_PitchAngle = 0;
    float sp_YawAngle = 0;
    float sp_RollAngle = 0;

    // System outputs (get from IMU)
    float imu_RollRate = 0;
    float imu_PitchRate = 0;
    float imu_YawRate = 0;

    //IMU integration outputs
    float imu_RollAngle = 0;
    float imu_PitchAngle = 0;
    float imu_YawAngle = 0;

    // Derivative Gains for gyro stabalization (Set for desired pwm servo pulse width)
    float kd_Gyro_Roll = 0;
    float kd_Gyro_Pitch = 0;
    float kd_Gyro_Yaw = 0;

    // Derivative Gains for accelerometer stabalization (Set for desired pwm servo pulse width)
    float kd_Accel_Roll = 1;
    float kd_Accel_Pitch = 1;
    float kd_Accel_Yaw = 1;

    // Control Signals (Output compare value)
    int control_Roll;
    int control_Pitch;
    int control_Throttle;
    int control_Yaw;

    char switched = 0;
    char autoTrigger = 0;

    // on/off "switches" for features
    int accelStabilization = 0;
    int autoPilotOnOff = 0;

    //System constants
    float maxRollAngle = 60; // max allowed roll angle in degrees
    float maxPitchAngle = 60;
    float maxYawAngle = 20;

    VN100_initSPI();

    if (DEBUG) {
        int gainSelector = 0; //0=Roll, 1= Pitch, 2=Yaw
        int gainTrigger = 0; //0 = Saved Value, 1 = Edit Mode
        initIC(0b11111111);
        initOC(0b1111); //Initialize only Output Compare 1,2,3 and 4
        UART1_SendString("START OF CODE BEFORE WHILE");
    } else {
        initIC(0b1111);
        initOC(0b1111); //Initialize only Output Compare 1,2,3 and 4
    }
    
    sp_Range = UPPER_PWM - MIDDLE_PWM;

    while (1) {
        if (DEBUG) {
            //UART1_SendString("Hi My Name is Mitch");
            //UART1_SendString("Hi My Name is CHRIS");
        }
        /*****************************************************************************
         *****************************************************************************

                                    INPUT CAPTURE

         *****************************************************************************
         *****************************************************************************/

        int* icTimeDiff;
        icTimeDiff = getICValues();


            sp_RollRate = (icTimeDiff[0] - MIDDLE_PWM);
            sp_PitchRate = (icTimeDiff[1] - MIDDLE_PWM);
            sp_ThrottleRate = (icTimeDiff[2]);
            sp_YawRate = (icTimeDiff[3] - MIDDLE_PWM);

            sp_GearSwitch = icTimeDiff[4];
            sp_Type = icTimeDiff[5];
            sp_Value = icTimeDiff[6];
            sp_Switch = icTimeDiff[7];

        if (DEBUG) {


        }

        /*****************************************************************************
         *****************************************************************************

                                    IMU COMMUNICATION

         *****************************************************************************
         *****************************************************************************/

        float imuData[3];
        VN100_SPI_GetRates(0, &imuData);
        //Outputs in order: Roll,Pitch,Yaw
        imu_RollRate = imuData[0];
        imu_PitchRate = imuData[1];
        imu_YawRate = imuData[2];

        VN100_SPI_GetYPR(0, &imuData[YAW], &imuData[PITCH], &imuData[ROLL]);
        imu_YawAngle = imuData[YAW];
        imu_PitchAngle = imuData[PITCH];
        imu_RollAngle = imuData[ROLL];


        if (DEBUG) {
            char str[20];
            //sprintf(str,"%f",imuData[0]);
            UART1_SendString(str);
        }
        /*****************************************************************************
         *****************************************************************************

                                     CONTROL CODE

         *****************************************************************************
         *****************************************************************************/

        if (STABILIZATION) { // if we are using accelerometer based stabalization
            // convert sp_xxxRate to an angle (based on maxangle
            //             if (!autoPilotOnOff){ // if we are getting input from the controller
            // convert sp_xxxxRate to an sp_xxxxAngle in degrees
            //sp_YawAngle = sp_YawRate / (sp_Range / maxYawAngle);
            sp_RollAngle = sp_RollRate / (sp_Range / maxRollAngle);
            sp_PitchAngle = -sp_PitchRate / (sp_Range / maxPitchAngle);

            //             } else {
            //get autopilot requested angle, set sp_xxxxAngle based on autopilot request
            //          }

            // output to servos based on requested angle and actual angle (using a gain value)
            // we set this to sp_xxxxRate so that the gyros can do their thing aswell
            /* this is commented untill we get the controlSignalAccel() function working
             */
            //sp_YawRate = controlSignalAngles(sp_YawAngle, imu_YawAngle, kd_Accel_Yaw, -(UPPER_PWM - MIDDLE_PWM) / (maxYawAngle)) ;
            sp_RollRate = controlSignalAngles(sp_RollAngle, imu_RollAngle, kd_Accel_Roll, -(UPPER_PWM - MIDDLE_PWM) / (maxRollAngle)) ;
            sp_PitchRate = controlSignalAngles (sp_PitchAngle, imu_PitchAngle, kd_Accel_Pitch, -(UPPER_PWM - MIDDLE_PWM) / (maxPitchAngle));


        }

        if (!STABILIZATION) {
            control_Roll = sp_RollRate + MIDDLE_PWM;
            control_Pitch = sp_PitchRate + MIDDLE_PWM;
            control_Yaw = sp_YawRate + MIDDLE_PWM;
            control_Throttle = sp_ThrottleRate;
        } else {


            if (sp_Switch < 600) {
                if (sp_GearSwitch > 600) {
                    if (sp_Type < 684) {
                        kd_Gyro_Roll = (float) (sp_Value - LOWER_PWM) / (UPPER_PWM - LOWER_PWM) * 2;
                    } else if (sp_Type > 684 && sp_Type < 718) {
                        kd_Gyro_Pitch = (float) (sp_Value - LOWER_PWM) / (UPPER_PWM - LOWER_PWM) * 2;
                    } else if (sp_Type > 718) {
                        kd_Gyro_Yaw = (float) (sp_Value - LOWER_PWM) / (UPPER_PWM - LOWER_PWM) * 2;
                    }

                } else {
                    //THROTTLE
                    if (sp_ThrottleRate > ((UPPER_PWM - LOWER_PWM) * 0.2) + LOWER_PWM && autoTrigger) {
                        control_Throttle = (((float) (sp_Value - 514) / (float) (888 - 514) + 0.5) * (UPPER_PWM - LOWER_PWM)) + LOWER_PWM;
                        if (UPPER_PWM < control_Throttle)
                            control_Throttle = UPPER_PWM;
                    }

                }
                if (sp_ThrottleRate < ((UPPER_PWM - LOWER_PWM) * 0.2) + LOWER_PWM && switched != (sp_Switch < 600)) {
                    control_Throttle = LOWER_PWM;
                } else if (switched != (sp_Switch < 600)) {
                    autoTrigger = 1;
                }

            } else {
                control_Throttle = sp_ThrottleRate;
                autoTrigger = 0;
            }
            switched = (sp_Switch < 600);


            // Control Signals (Output compare value)
            control_Roll = controlSignal(sp_RollRate / SERVO_SCALE_FACTOR, imu_RollRate, kd_Gyro_Roll);
            control_Pitch = controlSignal(sp_PitchRate / SERVO_SCALE_FACTOR, imu_PitchRate, kd_Gyro_Pitch);
            control_Yaw = controlSignal(sp_YawRate / SERVO_SCALE_FACTOR, imu_YawRate, kd_Gyro_Yaw);

        }
        /*****************************************************************************
         *****************************************************************************

                                    OUTPUT COMPARE

         *****************************************************************************
         *****************************************************************************/
        if (DEBUG) {

        }
        if (control_Roll > UPPER_PWM)
            control_Roll = UPPER_PWM;
        if (control_Roll < LOWER_PWM)
            control_Roll = LOWER_PWM;

        if (control_Pitch > UPPER_PWM)
            control_Pitch = UPPER_PWM;
        if (control_Pitch < LOWER_PWM)
            control_Pitch = LOWER_PWM;

        if (control_Yaw > UPPER_PWM)
            control_Yaw = UPPER_PWM;
        if (control_Yaw < LOWER_PWM)
            control_Yaw = LOWER_PWM;



        //Double check ocPin
        setPWM(1, control_Roll);
        setPWM(2, control_Pitch);
        setPWM(3, control_Throttle);
        setPWM(4, control_Yaw);

//        sendTelemetryBlock(getDebugTelemetryBlock());
        asm("CLRWDT");
    }
}
