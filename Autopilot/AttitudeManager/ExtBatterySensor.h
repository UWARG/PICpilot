/* 
 * File:   External Battery Sensor
 * Author: Stephen Cholvat
 * Comments:
 * Revision history: 
 */
#ifndef EXTBATTERYSENSOR_H
#define	EXTBATTERYSENSOR_H


#ifdef	__cplusplus
extern "C" {
#endif


//Function Prototypes
//TODO: Add descriptions for each function
void initExtBatterySensor();
int getExtBatteryLevel();
void initExtBatteryADC();

#ifdef	__cplusplus
}
#endif

#endif	/* VOLTAGESENSOR_H */

