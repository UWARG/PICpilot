/* 
 * File:   StateMachine.h
 * Author: Chris Hajduk
 *
 * Created on June 9, 2015, 8:58 PM
 */

#ifndef STATEMACHINE_H
#define	STATEMACHINE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "main.h"
#include "AttitudeManager.h"
#include "InterchipDMA.h"
#include "net.h"
#include "VN100.h"
#include "cameraManager.h"

    extern int state;

#define IOMATRIX_SIZE 9

    void StateMachine(char* cond);
    int getMatrixValue(float* IOMatrix, int* valueMatrix);
    void forceStateMachineUpdate();

#ifdef	__cplusplus
}
#endif

#endif	/* STATEMACHINE_H */

