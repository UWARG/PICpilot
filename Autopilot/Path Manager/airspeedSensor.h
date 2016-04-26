/* 
 * File:   airspeed.h
 * Author: Chris Hajduk
 *
 * Created on June 5, 2014, 8:18 PM
 */

#ifndef AIRSPEEDSENSOR_H
#define	AIRSPEEDSENSOR_H

#ifdef	__cplusplus
extern "C" {
#endif

#define AIRSPEED_HISTORY 20


//Function Prototypes
//TODO: Add descriptions for each function
void initAirspeedSensor();
//float timeRemaining();
float getCurrentAirspeed();
void initAirspeedADC();

#ifdef	__cplusplus
}
#endif

#endif	/* AIRSPEEDSENSOR_H */

