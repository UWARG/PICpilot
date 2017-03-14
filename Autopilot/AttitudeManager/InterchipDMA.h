/*
 * File:   InterchipDMA.h
 * Author: Chris Hajduk
 *
 * Created on February 2, 2014, 2:10 PM
 */

//TODO: Clean up this H file and the corresponding C file

#ifndef INTERCHIPDMA_H
#define	INTERCHIPDMA_H

#include "main.h"

//Intercom pins
#define INTERCOM_1 PORTAbits.RA12 // output
#define INTERCOM_2 PORTAbits.RA13 // output
#define INTERCOM_3 PORTBbits.RB4 // input
#define INTERCOM_4 PORTBbits.RB5 // input

#define IC_DMA_PORT 1

//Function Prototypes
void init_DMA0();
void init_DMA1();
char isDMADataAvailable();

#endif	/* INTERCHIPDMA_H */

