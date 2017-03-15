/*
 * File:   InterchipDMA.h
 * Author: Chris Hajduk
 *
 * Created on February 2, 2014, 2:10 PM
 */


#ifndef INTERCHIPDMA_H
#define	INTERCHIPDMA_H

//Intercom pins
#if ATTITUDE_MANAGER
#define INTERCOM_1 PORTAbits.RA12 // output
#define INTERCOM_2 PORTAbits.RA13 // output
#define INTERCOM_3 PORTBbits.RB4 // input
#define INTERCOM_4 PORTBbits.RB5 // input

#elif PATH_MANAGER
#define INTERCOM_1 PORTBbits.RB4 // input
#define INTERCOM_2 PORTBbits.RB5 // input
#define INTERCOM_3 PORTAbits.RA12 // output
#define INTERCOM_4 PORTAbits.RA13 // output
#endif

#define IC_DMA_PORT 1

//Function Prototypes
void init_DMA0(char isAttMan);
void init_DMA1(char isAttMan);
char isDMADataAvailable();

#endif	/* INTERCHIPDMA_H */

