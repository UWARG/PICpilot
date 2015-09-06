/*
 * File:   InputCapture.h
 * Author: Chris Hajduk
 *
 * Created on March 4, 2013, 10:31 PM
 */

#ifndef INPUTCAPTURE_H
#define	INPUTCAPTURE_H

#ifdef	__cplusplus
extern "C" {
#endif

// Includes
#include "main.h"
//TODO: Finish these function prototype descriptions
//Function Prototypes
void initInputCapture(char initIC);
void initIC(char initIC);
void initTimer2();
unsigned int* getICValues();
unsigned int getICValue(unsigned char channel);

#ifdef	__cplusplus
}
#endif

#endif	/* INPUTCAPTURE_H */