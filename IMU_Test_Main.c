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
#include "UART2.h"
#include "FullInitialize.h"
#include "lcd.h"
#include "delay.h"
#include "3DMGX1.h"

//Perform random clock things that came with the Sample Code (Keep in code)
_FOSCSEL(FNOSC_FRC);			// Internal FRC oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_NONE);
								// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
								// OSC2 Pin Function: OSC2 is Clock Output
								// Primary Oscillator Mode: XT Crystanl

//Define variables for global use in the code
#define TRUE	1
#define FALSE	0


unsigned char IMU_Receive[11];
int IMU_ReceiveCount = 0;
int IMU_FullReceive = FALSE;
int IMU_Valid = FALSE;
int IMU_Receive_Trigger = FALSE;
double IMU_Roll = 0, IMU_Pitch = 0, IMU_Yaw = 0;



//    else if (IMU_Receive_Trigger == TRUE)
//    {
//            IMU_Receive[IMU_ReceiveCount] = U2RXREG;
//
//            if (IMU_ReceiveCount == 10)
//            {
//                IMU_ReceiveCount = 0;
//                IMU_FullReceive = TRUE;
//                IMU_Receive_Trigger = FALSE;
//            }
//            else
//                IMU_ReceiveCount++;
//
//    }
//        IFS1bits.U2RXIF = 0;
//}

void __attribute__ ((interrupt, no_auto_psv)) _U2TXInterrupt(void)
{
    while(U2STAbits.TRMT == 0);
    U2STAbits.TRMT = 0;
    IFS1bits.U2TXIF = 0;
        //U2TXREG = 'a';
}

void CalcEulerAngles()
{
	unsigned short sRoll, sPitch, sYaw, ttick, checkS, checkV;
	short ssRoll, ssPitch, ssYaw;
	float align = ((float)360/(float)65536);

        sRoll = (IMU_Receive[1]*256)+ IMU_Receive[2];  //Convert Roll to a 16 bit unsigned integer
        if(sRoll > 32767)                    //Check for rollover and adjust
                ssRoll = sRoll - 65536;
        else ssRoll = sRoll;

        sPitch = (IMU_Receive[3]*256)+ IMU_Receive[4];
        if(sPitch > 32767)
                ssPitch = sPitch - 65536;
        else ssPitch = sPitch;

        sYaw = (IMU_Receive[5]*256)+ IMU_Receive[6];
        if(sYaw > 32767)
                ssYaw = sYaw - 65536;
        else ssYaw = sYaw;

        ttick = (IMU_Receive[7]*256)+ IMU_Receive[8];            //Timer Ticks
        checkS = (IMU_Receive[9]*256)+ IMU_Receive[10];         //Checksum value from device

        checkV = 14+sRoll+sPitch+sYaw+ttick;  //add up the 16 bit unsigned ints (roll, pitch, yaw, timer ticks) and cmd (0x0E)

        //Calculate the Roll/Pitch?Yaw values only if the checksum returned to be correct
        if (checkV == checkS)
        {
            IMU_Valid = TRUE;
            IMU_Roll = (double)ssRoll*align;   //apply align == 360/65536
            IMU_Pitch = (double)ssPitch*align;
            IMU_Yaw = (double)ssYaw*align;
        }
}


int main()
{
    FullInit();

    LATA = 1;

//    U2TXREG = 0x10;
//    while(U2STAbits.TRMT == 0);
//    U2STAbits.TRMT = 0;
//
//    U2TXREG = 0x00;
//    while(U2STAbits.TRMT == 0);
//    U2STAbits.TRMT = 0;
//
//    U2TXREG = 0x0E;
//    while(U2STAbits.TRMT == 0);
//    U2STAbits.TRMT = 0;
    LATA = 2;
    //init_3DMGX1();
    
    //struct IMU data;
    init_3DMGX1();
    struct IMU data;
    while(1)
    {


        int command = sGyroEulerAngles;
        updateCurrentData(&command,1);
        data = getCurrentData();
       // printf("%d",data.roll);
        Delay(Delay_15mS_Cnt);
        LATA = data.roll;

        //Calculate and Update the Input Values
        short ic;
        for (ic = 1; ic <= 8; ic++)
        {
            //If knew data has been received on the given channel
            if (checkic[ic] == 1)
            {
              //If the second time is greater than the first time then we have not met roll over
              if(t2[ic] > t1[ic])
              {
                 icTimeDiff[ic] = (t2[ic] - t1[ic]);
                 checkic[ic] = 0;
              }

              //Roll over has been made, therefore do math to find the time difference
              //(Max time - t1) is the upper difference
              //Adding the second time (time from 0) gives you the total time difference
              else
              {
                 icTimeDiff[ic] = ((PR2 - t1[ic]) + t2[ic]);
                 checkic[ic] = 0;
              }
            }
        }


        //If we've received a full set of information form the IMU, then Calculate the Euler Angles
        if (IMU_FullReceive == TRUE)
        {
            CalcEulerAngles();

            //If the checksum of the IMU received message returned valid
            //Update the control values
            if (IMU_Valid == TRUE)
            {
                int intRoll = Roll;
                LATA = intRoll;
                //Delay(Delay_5mS_Cnt);

                //Quadcopter input mixing
                //Output to: OCXRS = .... (X is output channel)

                int motor1, motor2, motor3, motor4; //Front Right, Back Left, Front Left, Back Right

                //Throttle
                motor1 = Throttle;
                motor2 = Throttle;
                motor3 = Throttle;
                motor4 = Throttle;

                //Roll - left roll (counter clockwise) is negative, right roll (clockwise) is positive
                motor2 += (Roll - MIDDLE_PWM)/2;
                motor3 += (Roll - MIDDLE_PWM)/2;
                motor1 += (MIDDLE_PWM - Roll)/2;
                motor4 += (MIDDLE_PWM - Roll)/2;

                //Pitch - Forward rotation is positive, backwards rotation is negative
                motor1 += (MIDDLE_PWM - Pitch)/2;
                motor3 += (MIDDLE_PWM - Pitch)/2;
                motor2 += (Pitch - MIDDLE_PWM)/2;
                motor4 += (Pitch - MIDDLE_PWM)/2;


                //Yaw - left (counter clockwise) is negative, right (clockwise) is positive
                motor1 += (Yaw - MIDDLE_PWM)/2;
                motor2 += (Yaw - MIDDLE_PWM)/2;
                motor3 += (MIDDLE_PWM - Yaw)/2;
                motor4 += (MIDDLE_PWM - Yaw)/2;


                init_EasyVarNames();
                //setPWM(1,icTimeDiff[1]);
            }

            //If the checksum turned out to be invalid
            else
                LATA = 2;
        }
    }
    return (EXIT_SUCCESS);
}


/*****************************************************************************
 *****************************************************************************

                             STABILIZATION CODE

 *****************************************************************************
 *****************************************************************************

//Servo scale factor is used in converting deg/s rotation input into output compare values
const float SERVO_SCALE_FACTOR = -(UPPER_PWM-MIDDLE_PWM)/45;

int controlSignal(float setpoint, float output, float gain) {
    int control = SERVO_SCALE_FACTOR*(setpoint-output)*gain + MIDDLE_PWM;
    return control;
}

int main() {

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
    int control_Roll = (sp_RollRate, imu_RollRate, kd_Roll);
    int control_Pitch = (sp_PitchRate, imu_PitchRate, kd_Pitch);
    int control_Yaw = (sp_YawRate, imu_YawRate, kd_Yaw);

}

 *****************************************************************************
 *****************************************************************************
 ******************************************************************************/
