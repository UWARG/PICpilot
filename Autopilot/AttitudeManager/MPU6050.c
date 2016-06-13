/*
 * File:   MPU6050.c
 * Author: Chris Hajduk
 *
 * Created on June 13, 2016, 6:05 PM
 */

#include "I2C.c"
#include "MPU6050.h"
#include "main.h"

// At powerup all registers should be zero except for:
//      Register 0x75 (WHO_AM_I)   = 0x68.
char imuConnected = 0;
MPUData deviceData;

char init_MPU6050(){
    initI2C();
    imuConnected = checkDevicePresence(I2C_SLAVE_ADDRESS, MPU6050_WHO_AM_I);

    // default at power-up:
    //    Gyro at 250 degrees second
    //    Acceleration at 2g
    //    Clock source at internal 8MHz
    //    The device is in sleep mode.
    //

    // Clear sleep mode
    char message = 0;
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_PWR_MGMT_1,&message, 1, WRITE);
    return imuConnected;
}

void getAccel(){
    char dataH, dataL;
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_ACCEL_XOUT_H, &dataH, 1, READ);
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_ACCEL_XOUT_L, &dataL, 1, READ);
    deviceData.accelX = ((int)dataH << 8) + dataL;
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_ACCEL_YOUT_H, &dataH, 1, READ);
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_ACCEL_YOUT_L, &dataL, 1, READ);
    deviceData.accelY = ((int)dataH << 8) + dataL;
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_ACCEL_ZOUT_H, &dataH, 1, READ);
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_ACCEL_ZOUT_L, &dataL, 1, READ);
    deviceData.accelZ = ((int)dataH << 8) + dataL;
}

void getGyro(){
    char dataH, dataL;
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_GYRO_XOUT_H, &dataH, 1, READ);
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_GYRO_XOUT_L, &dataL, 1, READ);
    deviceData.gyroX = ((int)dataH << 8) + dataL;
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_GYRO_YOUT_H, &dataH, 1, READ);
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_GYRO_YOUT_L, &dataL, 1, READ);
    deviceData.gyroY = ((int)dataH << 8) + dataL;
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_GYRO_ZOUT_H, &dataH, 1, READ);
    sendMessage(I2C_SLAVE_ADDRESS, MPU6050_GYRO_ZOUT_L, &dataL, 1, READ);
    deviceData.gyroZ = ((int)dataH << 8) + dataL;
}