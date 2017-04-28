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
        char data[4] = {0xB8, 0x07, 0x39, 0xB9};                                        //Data to set registers
        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[0], 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_EVENT_REGISTER, &data[1], 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[2], 1, WRITE);          //Set in Barometer mode
        
        int msb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_MSB_REGISTER, 0, 0, READ);    //Get Barometer Pressure Value
        int csb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_CSB_REGISTER, 0, 0, READ);
        int lsb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_LSB_REGISTER, 0, 0, READ);
        
        long int pressure_whole = (long)msb<<16 | (long)csb<<8 | (long)lsb;
        pressure_whole >>=6;
        
        lsb &= 0b00110000;
        lsb >>= 4;
        
        float pressure_decimal = (float)lsb/4.0;
        
        
        float pressure = (float)pressure_whole + pressure_decimal;
        
        sendMessage(I2C_SLAVE_ADDRESS, OFF_P, pressure - 101326, 1, WRITE);             //Set Offset as difference between measured value and sea level
        
        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[3], 1, WRITE);          //Switch to Altimeter mode

    }
    else{
#if DEBUG
        debug("Altimeter Device Not Connected!");
#endif
    }
    return altimeterConnected;
}

void calibrateAltimeter(float altitude){
    altimeterOffset = altitude;
}

float getAltitude() {
    if (altimeterConnected && (sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_REGISTER, 0, 0, READ) & 0x08)) {
        int msb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_MSB_REGISTER, 0, 0, READ) << 8;
        unsigned char csb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_CSB_REGISTER, 0, 0, READ);
        float lsb = (sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_LSB_REGISTER, 0, 0, READ) >> 4)/16.0;
        lastKnownAltitude = (float) (msb | csb) + lsb;                                                  //No altitude offset
    }
    return lastKnownAltitude;
}