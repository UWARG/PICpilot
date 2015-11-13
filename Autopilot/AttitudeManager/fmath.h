/* 
 * File:   fmath.h
 * Author: Chris Hajduk
 *
 * Created on April 12, 2014, 9:53 AM
 */

#ifndef FMATH_H
#define	FMATH_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "main.h"

#define SINE_TABLE_SIZE 256 ///Must be a power of 2

void initTrigLookup();
float lookup(int val);
float fSin(float val);
float fCos(float val);
float fTan(float val);

#ifdef	__cplusplus
}
#endif

#endif	/* FMATH_H */

