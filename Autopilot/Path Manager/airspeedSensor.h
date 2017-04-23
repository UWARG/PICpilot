/* 
 * File:   airspeed.h
 * Author: Chris Hajduk
 *
 * Created on June 5, 2014, 8:18 PM
 */

#ifndef AIRSPEEDSENSOR_H
#define	AIRSPEEDSENSOR_H

/**
 * Calibrates the airspeed sensor. Samples the sensor and 
 * averages them to get an offset for the reading.
 */
void calibrateAirspeed();

/**
 * Initializes the airspeed sensor
 */
void initAirspeedSensor();

/**
 * Reads the sensor history, averages it, and applies a transfer function
 * @return The current airspeed, in m/s
 */
float getCurrentAirspeed();

#endif	/* AIRSPEEDSENSOR_H */

