/* 
 * File:   IMU_Test_Main.c
 * Author: Mitch
 *
 * Created on June 15, 2013, 3:40 PM
 */ 

//Include Libraries
#include <stdio.h>
#include <stdlib.h>
#include <p33FJ256GP710A.h>
#include <string.h>

//Include the Full Initialization Header File
#include "Clock.h"
#include "FullInitialize.h"
#include "delay.h"
#include "VN100.h"
#include "InputCapture.h"
#include "OutputCompare.h"

//Perform random clock things that came with the Sample Code (Keep in code)
_FOSCSEL(FNOSC_FRC);			// Internal FRC oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_NONE);
								// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
								// OSC2 Pin Function: OSC2 is Clock Output
								// Primary Oscillator Mode: XT Crystanl

//Define variables for global use in the code
#define TRUE	1
#define FALSE	0





 /*****************************************************************************
 *****************************************************************************

                             STABILIZATION CODE

 *****************************************************************************
 *****************************************************************************/

//Servo scale factor is used in converting deg/s rotation input into output compare values
float SERVO_SCALE_FACTOR;

int controlSignal(float setpoint, float output, float gain) {
    int control = SERVO_SCALE_FACTOR*(setpoint-output)*gain + MIDDLE_PWM;
    return control;
}


/*****************************************************************************
 *****************************************************************************
 ******************************************************************************/

int main()
{
   SERVO_SCALE_FACTOR = -(UPPER_PWM-MIDDLE_PWM)/45;

    // Setpoints (From radio transmitter or autopilot)
    float sp_RollRate;
    float sp_YawRate;
    float sp_PitchRate;

    // System outputs (get from IMU)
    float imu_RollRate;
    float imu_PitchRate;
    float imu_YawRate;

    // Derivative Gains (Set for desired pwm servo pulse width)
    float kd_Roll;
    float kd_Pitch;
    float kd_Yaw;

    // Control Signals (Output compare value)
    int control_Roll;
    int control_Pitch;
    int control_Yaw;

    initOC();
    initIC();
    VN100_initSPI();
    
    LATA = 1;

    while(1)
    {
 /*****************************************************************************
 *****************************************************************************

                             INPUT CAPTURE

 *****************************************************************************
 *****************************************************************************/
        
    int* icTimeDiff;
    icTimeDiff = getICValues();

    sp_RollRate = icTimeDiff[3];
    sp_YawRate = icTimeDiff[5];
    sp_PitchRate = icTimeDiff[4];

 /*****************************************************************************
 *****************************************************************************

                             IMU COMMUNICATION

 *****************************************************************************
 *****************************************************************************/
      

/*****************************************************************************
 *****************************************************************************

                             CONTROL CODE

 *****************************************************************************
 *****************************************************************************/
    // Control Signals (Output compare value)
    control_Roll = controlSignal(sp_RollRate, imu_RollRate, kd_Roll);
    control_Pitch = controlSignal(sp_PitchRate, imu_PitchRate, kd_Pitch);
    control_Yaw = controlSignal(sp_YawRate, imu_YawRate, kd_Yaw);

 /*****************************************************************************
 *****************************************************************************

                             OUTPUT COMPARE

 *****************************************************************************
 *****************************************************************************/
    
    //Double check ocPin
    setPWM(3,control_Roll);
    setPWM(4,control_Pitch);
    setPWM(5,control_Yaw);
    }
}
