/* 
 * File:   MPL3115A2.h
 * Author: Chris Hajduk
 *
 * Created on April 5, 2014, 7:08 PM
 */
#include "../Common/Interfaces/I2C.h"

#ifndef MPL3115A2_H
#define	MPL3115A2_H

#ifdef	__cplusplus
extern "C" {
#endif


//The slave address is 0b1100000 for MPL3115A2.
#define I2C_SLAVE_ADDRESS 0b1100000

#define ALITUDE_CHECK_FLAG 2

#define DATA_READY_REGISTER 0
#define ALTITUDE_MSB_REGISTER 1
#define ALTITUDE_CSB_REGISTER 2
#define ALTITUDE_LSB_REGISTER 3
#define WHO_AM_I_REG 0x0C
#define DATA_READY_EVENT_REGISTER 0x13
#define BAR_IN_MSB 0x14
#define BAR_IN_LSB 0x15
#define CONTROL_REGISTER1 0x26
#define OFF_H 0x2D //offset register for alttitude
#define GIVEN_PRESSURE 97561 //current pressure



//TODO: ADD BETTER FORMATTING TO THIS HEADER FILE
char initAltimeter();
void calibrateAltimeter(int altitudeOffset);
float getAltitude();
void switchToBarometerMode(int toBarometer);
float getPressure();

#ifdef	__cplusplus
}
#endif

#endif	/* MPL3115A2_H */

