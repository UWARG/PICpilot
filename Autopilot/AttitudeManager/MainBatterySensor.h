/* 
 * File:   voltageSensor.h
 * Author: Chris Hajduk
 *
 * Created on June 5, 2014, 8:18 PM
 */

#ifndef MAINBATTERYSENSOR_H
#define	MAINBATTERYSENSOR_H

#ifdef	__cplusplus
extern "C" {
#endif


//Function Prototypes
//TODO: Add descriptions for each function
void initMainBatterySensor();
int getMainBatteryLevel();
void initMainBatteryADC();

#ifdef	__cplusplus
}
#endif

#endif	/* MAINBATTERYSENSOR_H */
