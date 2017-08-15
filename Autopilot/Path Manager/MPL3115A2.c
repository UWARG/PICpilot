/*
 * File:   MPL3115A2.c
 * Author: Chris Hajduk
 *
 * Created on April 5, 2014, 7:08 PM
 */
#include <stdio.h>
#include "MPL3115A2.h"
#include "main.h"
#include "../Common/Utilities/Logger.h"

float lastKnownAltitude = 0;
char altimeterConnected = 0;
float altimeterOffset = 0;
char barometerConnected = 0;
float lastKnownPressure = 0;

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
    // set pressure 
}

void calibrateAltimeter(int altitudeOffset){
    if (altitudeOffset < -128 || altitudeOffset > 128){
        error("Invalid altitude offset!");
        // the range of altitude offset should be within -128 to 128
    }
    
    else{
        switchMode(1); //Barometer mode
        float pressureFloat = 0.0;
        //There's slight delay on pressure display, wait until pressure is not zero
        while(pressureFloat <= 0.1){
            pressureFloat = getPressure();
        }
        uint32_t pressure = (int) pressureFloat;
        switchMode(0); //Altimeter mode

        // set ground pressure
        pressure = pressure << 1; //Value is input in 2 Pascal units
        int msb = pressure >> 8;
        int lsb = pressure & 0xFF;

        sendMessage(I2C_SLAVE_ADDRESS, BAR_IN_MSB, (char*)&msb, 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, BAR_IN_LSB, (char*)&lsb, 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, OFF_H, (char*)&altitudeOffset, 1, WRITE);
    }
}

float getAltitude() {
    if (altimeterConnected && (sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_REGISTER, 0, 0, READ) & 0x08)) {
        uint8_t pre_msb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_MSB_REGISTER, 0, 0, READ);
        uint8_t pre_csb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_CSB_REGISTER, 0, 0, READ);
        uint8_t pre_lsb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_LSB_REGISTER, 0, 0, READ);

        int msb = pre_msb << 8;
        uint8_t csb = pre_csb;
        float lsb = (pre_lsb >> 4)/16.0;
        lastKnownAltitude = (float) (msb | csb) + lsb;
 
    }
    return lastKnownAltitude;
}

// toBarometer = 1 -> switch to Barometer
// toBarometer = 0 -> switch to Altimeter
void switchMode(int toBarometer) {
    if (altimeterConnected){
        char data[3] = {0x38, 0x07, 0x39}; //switch to barometer mode
        if(!toBarometer) {
            data[0] = 0xB8;
            data[2] = 0xB9;
        }
        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[0], 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_EVENT_REGISTER, &data[1], 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[2], 1, WRITE);
    }
}


float getPressure() {
    if (altimeterConnected && (sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_REGISTER, 0, 0, READ) & 0x08)) {
        uint8_t pre_msb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_MSB_REGISTER, 0, 0, READ);
        uint8_t pre_csb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_CSB_REGISTER, 0, 0, READ);
        uint8_t pre_lsb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_LSB_REGISTER, 0, 0, READ);

       
        //In barometer mode, this is the way to read current pressure by using the altimeter 
        uint32_t msb = pre_msb;// convert into long int 32bit
        msb = msb << 10;// shift left 10
        uint16_t csb = pre_csb << 2;
        float lsb = (pre_lsb >> 4) / 4.0;
        lastKnownPressure = (float) (msb | csb) + lsb;
    }
    return lastKnownPressure;
}