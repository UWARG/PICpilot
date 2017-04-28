/* 
 * File:   StateMachine.h
 * Author: Chris Hajduk
 *
 * Created on June 9, 2015, 8:58 PM
 */

#ifndef STATEMACHINE_H
#define	STATEMACHINE_H

#define STATEMACHINE_MAINLOOP 0
#define STATEMACHINE_IDLE 1

#include "main.h"
#include "AttitudeManager.h"
#include "VN100.h"

void StateMachine(char entryLocation);
void killPlane(char action);

#endif	/* STATEMACHINE_H */
