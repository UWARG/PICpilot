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
double Roll = 0, Pitch = 0, Yaw = 0;

void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void)
{

    if (IMU_Receive_Trigger == FALSE)
    {
        int RegTest = 0;
        RegTest = U2RXREG;
        
        if (RegTest == 14)
        {
            IMU_Receive[IMU_ReceiveCount] = RegTest;
            IMU_ReceiveCount++;
            IMU_Receive_Trigger = TRUE;
        }
    }

    else if (IMU_Receive_Trigger == TRUE)
    {
            IMU_Receive[IMU_ReceiveCount] = U2RXREG;

            if (IMU_ReceiveCount == 10)
            {
                IMU_ReceiveCount = 0;
                IMU_FullReceive = TRUE;
                IMU_Receive_Trigger = FALSE;
            }
            else
                IMU_ReceiveCount++;

    }
        IFS1bits.U2RXIF = 0;
}

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

        Roll = (double)ssRoll*align;   //apply align == 360/65536
        Pitch = (double)ssPitch*align;
        Yaw = (double)ssYaw*align;

        checkV = 14+sRoll+sPitch+sYaw+ttick;  //add up the 16 bit unsigned ints (roll, pitch, yaw, timer ticks) and cmd (0x0E)
        if (checkV == checkS)
                IMU_Valid = TRUE;
}



int main()
{
    FullInit();

    /* set LEDs (D3-D10/RA0-RA7) drive state low */
    LATA = 0xFF00;
    /* set LED pins (D3-D10/RA0-RA7) as outputs */
    TRISA = 0xFF00;

    LATA = 1;

    U2TXREG = 0x10;
    while(U2STAbits.TRMT == 0);
    U2STAbits.TRMT = 0;

    U2TXREG = 0x00;
    while(U2STAbits.TRMT == 0);
    U2STAbits.TRMT = 0;

    U2TXREG = 0x0E;
    while(U2STAbits.TRMT == 0);
    U2STAbits.TRMT = 0;

    while(1)
    {
        if (IMU_FullReceive == TRUE)
        {
            CalcEulerAngles();

            if (IMU_Valid == TRUE)
            {
                int intRoll = Roll;
                LATA = intRoll;
                Delay(Delay_5mS_Cnt);
            }

            else
                LATA = 2;
        }
    }
    return (EXIT_SUCCESS);
}
