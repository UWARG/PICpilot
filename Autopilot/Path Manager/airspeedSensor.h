/* 
 * File:   airspeed.h
 * Author: Chris Hajduk
 *
 * Created on June 5, 2014, 8:18 PM
 */

#ifndef AIRSPEEDSENSOR_H
#define	AIRSPEEDSENSOR_H

#define AIRSPEED_HISTORY 20

//Function Prototypes
//TODO: Add descriptions for each function
void calibrateAirspeed();
void initAirspeedSensor();
static float ADCConvert(float signal);
float getCurrentAirspeed();
void initAirspeedADC();

#endif	/* AIRSPEEDSENSOR_H */

