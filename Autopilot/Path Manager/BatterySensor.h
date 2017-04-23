/* 
 * File:   voltageSensor.h
 * Author: Chris Hajduk
 *
 * Created on June 5, 2014, 8:18 PM
 */

#ifndef VOLTAGESENSOR_H
#define	VOLTAGESENSOR_H

/**
 * Initializes the ADC for both batteries
 */
void initBatterySensor();

/**
 * Retrieves the battery level of the main (PICpilot) battery
 * @return The voltage of the main battery * 100
 */
uint16_t getMainBatteryLevel();

/**
 * Retrieves the battery level of the external (Motor) battery
 * @return The voltage of the external battery * 100
 */
uint16_t getExtBatteryLevel();

#endif	/* VOLTAGESENSOR_H */

