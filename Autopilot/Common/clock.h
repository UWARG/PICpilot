/* 
 * File:   clock.h
 * Author: Chris Hajduk
 *
 * Created on September 8, 2015, 11:16 PM
 */

#ifndef CLOCK_H
#define	CLOCK_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#define CLOCK_FREQUENCY 40000000

void useFRCPLLClock();


#ifdef	__cplusplus
}
#endif

#endif	/* CLOCK_H */

