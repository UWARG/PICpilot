/*
 * File:   MPL3115A2.c
 * Author: Chris Hajduk
 *
 * Created on April 5, 2014, 7:08 PM
 */
#include "MPL3115A2.h"
#include "main.h"
#include "../Common/Utilities/Logger.h"

float lastKnownAltitude = 0;
char altimeterConnected = 0;
float altimeterOffset = 0;

char initAltimeter() {
    initI2C();
    altimeterConnected = checkDevicePresence(I2C_SLAVE_ADDRESS,WHO_AM_I_REG);
    if (altimeterConnected){
        
        //char data[3] = {0x38, 0x07, 0x39}; //switch to barometer mode
        char data[3] = {0xB8, 0x07, 0xB9};//switch to altimeter mode

        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[0], 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_EVENT_REGISTER, &data[1], 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[2], 1, WRITE);
    }
    else{
#if DEBUG
        debug("Altimeter Device Not Connected!");
#endif
    }
    return altimeterConnected;
}

void calibrateAltimeter(long int pressure){
    int msb = pressure >> 9;
    int lsb = (pressure >> 1)& 255;
    int offset = 0;
    sendMessage(I2C_SLAVE_ADDRESS, BAR_IN_MSB, (char*)&msb, 1, WRITE);
    sendMessage(I2C_SLAVE_ADDRESS, BAR_IN_LSB, (char*)&lsb, 1, WRITE);
    sendMessage(I2C_SLAVE_ADDRESS, OFF_H, (char*)&offset, 1, WRITE);
}

float getAltitude() {
    if (altimeterConnected && (sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_REGISTER, 0, 0, READ) & 0x08)) {
        unsigned char pre_msb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_MSB_REGISTER, 0, 0, READ);
        unsigned char pre_csb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_CSB_REGISTER, 0, 0, READ);
        unsigned char pre_lsb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_LSB_REGISTER, 0, 0, READ);

       /* 
        //In barometer mode, this is the way to read current pressure by using the altimeter 
        unsigned long int msb = pre_msb;// convert into long int 32bit
        msb = msb << 10;// shit left 10
        unsigned int csb = pre_csb << 2;
        float lsb = (pre_lsb >> 4) / 4.0;
        lastKnownAltitude = (float) (msb | csb) + lsb;
        */

        int msb = pre_msb << 8;
        unsigned char csb = pre_csb;
        float lsb = (pre_lsb >> 4)/16.0;
        lastKnownAltitude = (float) (msb | csb) + lsb - altimeterOffset;
    }
    return lastKnownAltitude;
}