/* 
 * File:   ProbeDrop.h
 * Author: banky
 *
 * Created on January 18, 2016, 8:10 PM
 */

#ifndef PROBEDROP_H
#define	PROBEDROP_H

#include "Dubins.h"
void getVelocityOfWind(float* groundVelocity, float* windVelocity, float* velocityOfWind);
char probeDrop(char verifiedDrop, Vector* targetPosition, float* currentPosition, float* altitude, float* groundVelocity, float* windVelocity);
#endif	/* PROBEDROP_H */

