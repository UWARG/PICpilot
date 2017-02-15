/* 
 * File:   StateMachine.h
 * Author: Chris Hajduk
 *
 * Created on June 9, 2015, 8:58 PM
 */

#ifndef STATEMACHINE_H
#define	STATEMACHINE_H

#define STATEMACHINE_MAINLOOP 0
#define STATEMACHINE_IMU 1
#define STATEMACHINE_IDLE 2

#define DMA_UPDATE_FREQUENCY 10

#include "main.h"
#include "AttitudeManager.h"
#include "InterchipDMA.h"
#include "net.h"
#include "VN100.h"
#include "cameraManager.h"

void StateMachine(char entryLocation);
void forceStateMachineUpdate();
void killPlane(char action);

#endif	/* STATEMACHINE_H */
