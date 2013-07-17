
#include "3DMGX1.h"
//Include Libraries
#include <stdio.h>
#include <stdlib.h>
#include <p33FJ256GP710A.h>
#include <string.h>
//Include the Full Initialization Header File
#include "UART2.h"
#include "delay.h"

/*
 * File:   3DMGX1.c
 * Author: Chris Hajduk
 *
 * Created on July 7, 2013, 4:40 PM
 */

/*
 * Use this file as a modular 3DMGX1 code base for common IMU functions.
 *
 */

const int messageSize[] = {0,23,23,23,13,13,6,7,2,2,23,23,31,11,11,5,7,5,31,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,11,0,5,7,7,0,0,0,0,0,0,0,23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,23,11};


int test = 0;
unsigned char IMU_Receive[23];
int IMU_ReceiveCount = 0;
int IMU_FullReceive = FALSE;
int IMU_Valid = FALSE;
int IMU_Receive_Trigger = FALSE;
int IMU_CurrentCommand = 0;
int gotGainScales = 0;
int initializing = 0;

struct IMU model;
double vector[3];

void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void)
{
//        home_clr();
//        char ReceiveMsg[] = "Receiving";
//        SendToLCD( (unsigned char*) &ReceiveMsg, sizeof(ReceiveMsg) -1 );
//        Delay(Delay_1S_Cnt);
//        int ReadInCount = 0;
//        while (ReadInCount < 3)
//        {

    if (IMU_Receive_Trigger == FALSE)
    {
        int RegTest = 0;
        RegTest = U2RXREG;

        //Checks if the IMU responded correctly (ie. header = return command issued)
        if (RegTest == IMU_CurrentCommand)
        {
            IMU_Receive[IMU_ReceiveCount++] = RegTest;
            IMU_Receive_Trigger = TRUE;
            //LATA = 8;
        }
    }

    else if (IMU_Receive_Trigger == TRUE)
    {
            IMU_Receive[IMU_ReceiveCount] = U2RXREG;

            if (IMU_CurrentCommand > 0x42 && IMU_ReceiveCount == 4) //5 byte response for an unrecognized command
            {
                IMU_ReceiveCount = 0;
                IMU_FullReceive = TRUE;
                IMU_Receive_Trigger = FALSE;
                //LATA = 0xFF;
            }
            else if (IMU_ReceiveCount == messageSize[IMU_CurrentCommand] - 1)
            {
                IMU_ReceiveCount = 0;
                IMU_FullReceive = TRUE;
                IMU_Receive_Trigger = FALSE;
                //LATA = 0xFF;
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
}

double* calcEulerAngles()
{

    /*
     * Data provided from IMU_Receive in this specification:
     * -ALL 16 bit signed integers
     *
     * imuData[1] = Roll MSB
     * imuData[2] = Roll LSB
     * imuData[3] = Pitch MSB
     * imuData[4] = Pitch LSB
     * imuData[5] = Yaw MSB
     * imuData[6] = Yaw LSB
     */

	unsigned short sRoll, sPitch, sYaw, ttick, checkS;
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

        vector[0] = (double)ssRoll*align;   //apply align == 360/65536
        vector[1] = (double)ssPitch*align;
        vector[2] = (double)ssYaw*align;
        return &vector;
}
double* calcAccelRate(){
    //Acceleration defined in G's (9.81m/s^2)
     /*
     * Data provided from IMU_Receive in this specification:
     * -ALL 16 bit signed integers
     *
     * imuData[7] = Accel_X MSB
     * imuData[8] = Accel_X LSB
     * imuData[9] = Accel_Y MSB
     * imuData[10] = Accel_Y LSB
     * imuData[11] = Accel_Z MSB
     * imuData[12] = Accel_Z LSB
     */

    //AccelGainScale is defined at EEPROM address #230
    float align = 32768000/ model.accelGainScale;
    unsigned short sAccelX,sAccelY,sAccelZ;

    sAccelX = IMU_Receive[7] * 256 + IMU_Receive[8];
    if (sAccelX > 32767)
        sAccelX -= 65535;
    sAccelY = IMU_Receive[9] * 256 + IMU_Receive[10];
    if (sAccelY > 32767)
        sAccelY -= 65535;
    sAccelZ = IMU_Receive[11] * 256 + IMU_Receive[12];
    if (sAccelZ > 32767)
        sAccelZ -= 65535;

    vector[0] = (double)sAccelX / align;
    vector[1] = (double)sAccelY / align;
    vector[2] = (double)sAccelZ / align;

    return &vector;
}
double* calcAngRate(){
    //Angular Velocity defined as Rad/Sec.
     /*
     * Data provided from IMU_Receive in this specification:
     * -ALL 16 bit signed integers
     *
     * imuData[13] = Accel_X MSB
     * imuData[14] = Accel_X LSB
     * imuData[15] = Accel_Y MSB
     * imuData[16] = Accel_Y LSB
     * imuData[17] = Accel_Z MSB
     * imuData[18] = Accel_Z LSB
     */

    float align = 32768000/ model.gyroGainScale;
    unsigned short sAngRateX, sAngRateY, sAngRateZ;
    

    sAngRateX = IMU_Receive[13] * 256 + IMU_Receive[14];
    if (sAngRateX > 32767)
        sAngRateX -= 65535;
    sAngRateY = IMU_Receive[15] * 256 + IMU_Receive[16];
    if (sAngRateY > 32767)
        sAngRateY -= 65535;
    sAngRateZ = IMU_Receive[17] * 256 + IMU_Receive[18];
    if (sAngRateZ > 32767)
        sAngRateZ -= 65535;

    vector[0] = (double)sAngRateX / align;
    vector[1] = (double)sAngRateY / align;
    vector[2] = (double)sAngRateZ / align;

    return &vector;
}


int checkValidity(unsigned char* data){
    //Checks if last transmission was valid
    int len = messageSize[IMU_CurrentCommand];
    int gChecksum;
    gChecksum = (IMU_Receive[len - 2] *256) + IMU_Receive[len - 1];
    int cChecksum;
    int i = 0;
    //Assumes each piece of data has a MSB and a LSB, excluding the header
    cChecksum = IMU_Receive[0];
    for (i = 1; i < len - 2; i+=2){
        int msb = IMU_Receive[i];
        int lsb = IMU_Receive[i+1];
        cChecksum += (msb * 256) + lsb;
    }
    if (gChecksum == cChecksum){
        return 1;
    }
    else
        return 0;
}
struct IMU getCurrentData(){
    //Check to ensure validity, otherwise don't update the valid data with invalid data
    
    if (checkValidity(&IMU_Receive)){
        double* temp;
        temp = calcEulerAngles();
        model.roll = temp[0];
        model.pitch = temp[1];
        model.yaw = temp[2];
        //Check to ensure the Gain Scales are defined
        if (gotGainScales){
            temp = calcAccelRate();
            model.accel_x = temp[0];
            model.accel_y = temp[1];
            model.accel_z = temp[2];
            temp = calcAngRate();
            model.rate_x = temp[0];
            model.rate_y = temp[1];
            model.rate_z = temp[2];
        }
    }

    return model;
}

void updateCurrentData(char* command, int dataLength) {
        int i;
        IMU_CurrentCommand = command[0];
        for (i = 0; i < dataLength; i++) { //Char = 1 byte
            U2TXREG = command[i];
            while (U2STAbits.TRMT == 0);
            U2STAbits.TRMT = 0;
        }
}

void init_3DMGX1(){
    //Somewhere here initialize Gain Scales -- NOT SURE THIS CODE WORKS (IE. is the interrupt triggered with 2 bytes of data?) -- Might freez up here
    initializing = 1;
    char data1[] = {sReadEEPROM, 230}; //AccelGainScale at 230
    while(IMU_FullReceive == FALSE){
        Delay(Delay_1S_Cnt);
        updateCurrentData(&data1,2);
    }
    
    
    model.accelGainScale = IMU_Receive[0] * 256 + IMU_Receive[1];
    //printf("%d",model.accelGainScale);

    char data2[] = {sReadEEPROM, 130}; //GyroGainScale at 130
    while(IMU_FullReceive == FALSE){
        Delay(Delay_1S_Cnt);
        updateCurrentData(&data2,2);
    }
    //while(IMU_FullReceive == FALSE);
    model.gyroGainScale = IMU_Receive[0] * 256 + IMU_Receive[1];
    //printf("%d",model.gyroGainScale);
    gotGainScales = TRUE;

}
unsigned char* returnCommandData(){
    //Simply returns the data from the last command
    return &IMU_Receive;
}