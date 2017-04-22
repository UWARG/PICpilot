/* 
 * File:   voltageSensor.h
 * Author: Chris Hajduk
 *
 * Created on June 5, 2014, 8:18 PM
 */

#ifndef VOLTAGESENSOR_H
#define	VOLTAGESENSOR_H

void initBatterySensor();

uint16_t getMainBatteryLevel();

uint16_t getExtBatteryLevel();

#endif	/* VOLTAGESENSOR_H */

