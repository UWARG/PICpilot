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

//Function Prototypes
void initInputCapture(char initIC);
void initIC(char initIC);
void initTimer2();
int* getICValues();
int getICValue(char ic);

#ifdef	__cplusplus
}
#endif

#endif	/* INPUTCAPTURE_H */