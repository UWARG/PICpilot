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
#include "delay.h"
#include "VN100.h"
#include "InputCapture.h"
#include "OutputCompare.h"
#include "net.h"
#include "StartupErrorCodes.h"
#include "OrientationControl.h"

//Perform random clock things that came with the Sample Code (Keep in code)
_FOSCSEL(FNOSC_FRC); // Internal FRC oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);
// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystanl

long long time = 0;
long long lastTime = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void){
    //Temporary Timer Interrupt
    time += 20;
    /* Interrupt Service Routine code goes here */
    IFS0bits.T2IF = 0;
    // Clear Timer1 Interrupt Flag
}

int main() {



    //Debug Mode:
    if (DEBUG) {
        InitUART1();  
    }
    checkErrorCodes();

    initDataLink();

    // Setpoints (From radio transmitter or autopilot)
    float SERVO_SCALE_FACTOR = (-(UPPER_PWM - MIDDLE_PWM) / 45);
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
    //Integral Gains for accelerometer stabilization
    float ki_Accel_Pitch = 0.01;
    float ki_Accel_Roll = 0.01;
    //Integral Gains for accelerometer stabilization
    float sum_Accel_Pitch = 0;
    float sum_Accel_Roll = 0;

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
    //getAngleBias();
    VN100_SPI_Tare(0);

    if (DEBUG) {
        int gainSelector = 0; //0=Roll, 1= Pitch, 2=Yaw
        int gainTrigger = 0; //0 = Saved Value, 1 = Edit Mode
        initIC(0b11111111);
        initOC(0b1111); //Initialize only Output Compare 1,2,3 and 4
        UART1_SendString("START OF CODE BEFORE WHILE");
    } else {
        initIC(0b11111111);
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
        imu_RollRate = (imuData[ROLL_RATE]);
//        imu_PitchRate = imuData[PITCH_RATE];
//        imu_YawRate = imuData[YAW_RATE];
        imu_PitchRate = imuData[YAW_RATE];
        imu_YawRate = imuData[PITCH_RATE];

        VN100_SPI_GetYPR(0, &imuData[YAW], &imuData[PITCH], &imuData[ROLL]);
//        imu_YawAngle = imuData[YAW] - angle_zero[YAW];
//        imu_PitchAngle = imuData[PITCH] - angle_zero[PITCH];
//        imu_RollAngle = (imuData[ROLL] - angle_zero[ROLL]);
        imu_YawAngle = imuData[PITCH] - angle_zero[PITCH];
        imu_PitchAngle = imuData[YAW] - angle_zero[YAW];
        imu_RollAngle = (imuData[ROLL] - angle_zero[ROLL]);


        if (DEBUG) {
//            char str[20];
//            //sprintf(str,"%f",imuData[0]);
//            UART1_SendString(str);
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
            sp_RollRate = controlSignalAngles(sp_RollAngle, imu_RollAngle, ROLL, -(UPPER_PWM - MIDDLE_PWM) / (maxRollAngle)) ;
            sp_PitchRate = controlSignalAngles (sp_PitchAngle, imu_PitchAngle, PITCH, -(UPPER_PWM - MIDDLE_PWM) / (maxPitchAngle));


        }

        if (!STABILIZATION) {
            control_Roll = sp_RollRate + MIDDLE_PWM;
            control_Pitch = sp_PitchRate + MIDDLE_PWM;
            control_Yaw = sp_YawRate + MIDDLE_PWM;
            control_Throttle = sp_ThrottleRate;
        } else {


            if (sp_Switch < 600) {
                unfreezeIntegral();
                if (sp_GearSwitch > 600) {
                    if (sp_Type < 684) {
                        setGain(ROLL,GAIN_KD,((float) (sp_Value - LOWER_PWM)) / (UPPER_PWM - LOWER_PWM) * 20);
                    } else if (sp_Type > 684 && sp_Type < 718) {
                        setGain(PITCH,GAIN_KD,((float) (sp_Value - LOWER_PWM)) / (UPPER_PWM - LOWER_PWM) * 20);
                    } else if (sp_Type > 718) {
                        setGain(YAW,GAIN_KD,((float) (sp_Value - LOWER_PWM)) / (UPPER_PWM - LOWER_PWM) * 20);
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
                freezeIntegral();
            }
            switched = (sp_Switch < 600);


            // Control Signals (Output compare value)
            control_Roll = controlSignal(sp_RollRate / SERVO_SCALE_FACTOR, imu_RollRate,ROLL_RATE);
            control_Pitch = controlSignal(sp_PitchRate / SERVO_SCALE_FACTOR, imu_PitchRate, PITCH_RATE);
            control_Yaw = controlSignal(sp_YawRate / SERVO_SCALE_FACTOR, imu_YawRate, YAW_RATE);

        }
        /*****************************************************************************
         *****************************************************************************

                                    OUTPUT COMPARE

         *****************************************************************************
         *****************************************************************************/
        if (DEBUG) {

        }
        if (control_Roll > UPPER_PWM){
            control_Roll = UPPER_PWM;
            if (2 * getIntegralSum(ROLL) * getGain(ROLL,GAIN_KI) > sp_RollRate){
                setIntegralSum(ROLL,getIntegralSum(ROLL)/1.2);
            }
        }
        if (control_Roll < LOWER_PWM){
            control_Roll = LOWER_PWM;
            if (2 * getIntegralSum(ROLL) * getGain(ROLL,GAIN_KI) < sp_RollRate){
                setIntegralSum(ROLL,getIntegralSum(ROLL)/1.2);
            }
        }
        if (control_Pitch > UPPER_PWM){
            control_Pitch = UPPER_PWM;
            if (2 * getIntegralSum(ROLL) * getGain(ROLL,GAIN_KI) > sp_RollRate){
                setIntegralSum(ROLL,getIntegralSum(ROLL)/1.2);
            }
        }
        if (control_Pitch < LOWER_PWM){
            control_Pitch = LOWER_PWM;
            if (2 * getIntegralSum(ROLL) * getGain(ROLL,GAIN_KI) < sp_RollRate){
                setIntegralSum(ROLL,getIntegralSum(ROLL)/1.2);
            }
        }

        if (control_Yaw > UPPER_PWM)
            control_Yaw = UPPER_PWM;
        if (control_Yaw < LOWER_PWM)
            control_Yaw = LOWER_PWM;



        //Double check ocPin
        setPWM(1, control_Roll);
        setPWM(2, control_Pitch);
        setPWM(3, control_Throttle);
        setPWM(4, control_Yaw);

        if (time - lastTime > 500){
            lastTime = time;

            struct telem_block* statusData = createTelemetryBlock();
            (*statusData).millis = time;
            (*statusData).lat = 0;
            (*statusData).lon = 0;
            (*statusData).pitch = imu_PitchAngle;
            (*statusData).roll = imu_RollAngle;
            (*statusData).yaw = imu_YawAngle;
            (*statusData).pitchRate = imu_PitchRate;
            (*statusData).rollRate = imu_RollRate;
            (*statusData).yawRate = imu_YawRate;
            (*statusData).pitch_gain = getGain(PITCH,GAIN_KD);
            (*statusData).roll_gain = getGain(ROLL,GAIN_KD);
            (*statusData).yaw_gain = getGain(YAW,GAIN_KD);
            (*statusData).pitchSetpoint = sp_PitchRate;
            (*statusData).rollSetpoint = sp_RollRate;
            (*statusData).yawSetpoint = sp_YawRate;
            (*statusData).throttleSetpoint = control_Throttle;
            (*statusData).editing_gain = (sp_Switch < 600 && sp_GearSwitch > 600);

            sendTelemetryBlock(statusData);
            destroyTelemetryBlock(statusData);
        }
        asm("CLRWDT");
    }
}
