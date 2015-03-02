/*
 * File:   MPL3115A2.c
 * Author: Chris Hajduk
 *
 * Created on April 5, 2014, 7:08 PM
 */
#include "MPL3115A2.h"
#include "UART1.h"

float lastKnownAltitude = 0;
char altimeterConnected = 0;
float altimeterOffset = 0;

char initAltimeter() {
    initI2C();
    altimeterConnected = checkDevicePresence(I2C_SLAVE_ADDRESS,WHO_AM_I_REG);
    if (altimeterConnected){
        char data[3] = {0xB8, 0x07, 0xB9};
        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[0], 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_EVENT_REGISTER, &data[1], 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[2], 1, WRITE);
    }
    else{
        warning("Altimeter Device Not Connected!");
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
        lastKnownAltitude = (float) (msb | csb) + lsb - altimeterOffset;
    }
    return lastKnownAltitude;
}