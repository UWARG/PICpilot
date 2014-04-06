/* 
 * File:   MPL3115A2.h
 * Author: Chris Hajduk
 *
 * Created on April 5, 2014, 7:08 PM
 */
#include "I2C.h"

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
#define DATA_READY_EVENT_REGISTER 0x13
#define CONTROL_REGISTER1 0x26

//TODO: ADD BETTER FORMATTING TO THIS HEADER FILE
void initAltimeter();
float getAltitude();


#ifdef	__cplusplus
}
#endif

#endif	/* MPL3115A2_H */

