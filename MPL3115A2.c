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

void initAltimeter() {
    initI2C();
    altimeterConnected = checkDevicePresence(I2C_SLAVE_ADDRESS,WHO_AM_I_REG);
    if (altimeterConnected){
        char data[3] = {0xB8, 0x07, 0xB9};
        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[0], 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_EVENT_REGISTER, &data[1], 1, WRITE);
        sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[2], 1, WRITE);
    }
    else{
        UART1_SendString("Altimeter Device Not Connected!");
    }
}

void calibrateAltimeter(float altitude){
    //Put into pressure mode
    char data[3] = {0b00111011, 0b00111001, 0};
    sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[0], 1, WRITE);
    sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[1], 1, WRITE);
    int i = 0;
    float pressureData[4];
    unsigned long msbPressure;
    unsigned long csbPressure;
    unsigned long lsbPressure;
     char str[16];
    for (i = 0; i < 4; i++){

        while(!(sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_REGISTER, 0, 0, READ) & 0x08));
        msbPressure = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_MSB_REGISTER, 0, 0, READ);
        csbPressure = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_CSB_REGISTER, 0, 0, READ);
        lsbPressure = (float)(sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_LSB_REGISTER, 0, 0, READ)); //dividing by 4, since two lowest bits are fractional value
        long pData = (long)msbPressure<<16 | (long)csbPressure<<8 | (long)lsbPressure; //shifting 2 to the left to make room for LSB
        pData >>= 6;
        lsbPressure &= 0b00110000;
        lsbPressure >>= 4;
        pressureData[i] = (float)pData + ((float)lsbPressure/4.0);
            sprintf(&str, "%f", pressureData[i]);
    UART1_SendString(&str);
    }

    float avgPres = (pressureData[0] + pressureData[1] + pressureData[2] + pressureData[3])/4;
   
    sprintf(&str, "%f", avgPres);
    UART1_SendString(&str);

    float seapress = avgPres/pow(1-altitude*0.0000225577,5.255877);

    data[0] = (unsigned int)(seapress/2) >> 8;
    data[1] = (unsigned int)(seapress/2) & 0xFF;

    sendMessage(I2C_SLAVE_ADDRESS, BAR_IN_MSB, &data[0], 1, WRITE);
    sendMessage(I2C_SLAVE_ADDRESS, BAR_IN_LSB, &data[1], 1, WRITE);

    //Restore standard settings
    data[0] = 0xB8;
    data[1] = 0x07;
    data[2] = 0xB9;
    sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[0], 1, WRITE);
    sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_EVENT_REGISTER, &data[1], 1, WRITE);
    sendMessage(I2C_SLAVE_ADDRESS, CONTROL_REGISTER1, &data[2], 1, WRITE);
}

float getAltitude() {
    if (altimeterConnected && (sendMessage(I2C_SLAVE_ADDRESS, DATA_READY_REGISTER, 0, 0, READ) & 0x08)) {
        int msb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_MSB_REGISTER, 0, 0, READ) << 8;
        unsigned char csb = sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_CSB_REGISTER, 0, 0, READ);
        float lsb = (sendMessage(I2C_SLAVE_ADDRESS, ALTITUDE_LSB_REGISTER, 0, 0, READ) >> 4)/16.0;
        lastKnownAltitude = (float) (msb | csb) + lsb;
    }
    return lastKnownAltitude;
}