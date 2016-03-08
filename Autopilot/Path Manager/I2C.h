/* 
 * File:   I2C.h
 * Author: Chris Hajduk
 *
 * Created on March 10, 2013, 7:58 PM
 */

#ifndef I2C_H
#define I2C_H

#include "main.h"
//#include "delay.h"

#define READ 1
#define WRITE 0

#define I2CIdle() while((I2C2CON & 0x1F ) || I2C2STATbits.TRSTAT == 1);

char checkDevicePresence(char devAddress, char WHO_AM_I_REG);
char sendMessage(char devAddress, char address, char* data, char length, char rw);
void initI2C();
char readMessage(char devAddress, char address);
void writeMessage(char address, char* data, char length);

#endif