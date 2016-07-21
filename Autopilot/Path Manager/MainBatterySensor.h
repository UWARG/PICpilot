/* 
 * File:   voltageSensor.h
 * Author: Chris Hajduk
 *
 * Created on June 5, 2014, 8:18 PM
 */

#ifndef VOLTAGESENSOR_H
#define	VOLTAGESENSOR_H

#ifdef	__cplusplus
extern "C" {
#endif

//Function Prototypes
void initMainBatterySensor();
int getMainBatteryLevel();
void initMainBatteryADC();

#ifdef	__cplusplus
}
#endif

#endif	/* VOLTAGESENSOR_H */

