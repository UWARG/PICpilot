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


#define vTimeInterval 4.461/60

//Function Prototypes
//TODO: Add descriptions for each function
void initBatterySensor();
float timeRemaining();
char getCurrentPercent();
char initTimer();
char checkBattery();
void initADC();
int readADC();

#ifdef	__cplusplus
}
#endif

#endif	/* VOLTAGESENSOR_H */

