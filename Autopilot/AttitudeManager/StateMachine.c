/* 
 * File:   StateMachine.c
 * Author: Chris Hajduk
 *
 * Created on June 9, 2015, 9:00 PM
 */

#include "StateMachine.h"
#include "../Common/debug.h"
#include "../Common/Common.h"
#include "main.h"
#include "ProgramStatus.h"

/*
 * 
 */

//State Machine Triggers (Mostly Timers)
int dmaTimer = 0;
int uplinkTimer = 0;
int downlinkP0Timer = 0;
int downlinkP1Timer = 0;
int downlinkP2Timer = 0;
int imuTimer = 0;
char AMUpdate = 0;
char flightUpdate = 0;
long int stateMachineTimer = 0;
int dTime = 0;

void StateMachine(char entryLocation){
    //Timers
    dTime = (int)(getTime() - stateMachineTimer);
    stateMachineTimer = getTime();
    uplinkTimer += dTime;
    downlinkP0Timer += dTime;
    downlinkP1Timer += dTime;
    downlinkP2Timer += dTime;
    imuTimer += dTime;
    dmaTimer += dTime;
    
    
    //Clear Watchdog timer
    asm("CLRWDT");
    //Feedback systems such as this autopilot are very sensitive to timing. In order to keep it consistent we should try to keep the timing between the calculation of error corrections and the output the same.
    //In other words, roll pitch and yaw control, mixing, and output should take place in the same step.
    if(AMUpdate){
        AMUpdate = 0;
        //Run - Angle control, and angular rate control
        flightUpdate = 1;
    }
    else if(IMU_UPDATE_FREQUENCY <= imuTimer && entryLocation != STATEMACHINE_IMU){
//        debug("IMU");
        imuTimer = 0;
        //Poll Sensor
        imuCommunication();
        
        flightUpdate = 1;
    }
    else if(isDMADataAvailable() && checkDMA()){
        //Input from Controller
        flightUpdate = 1;
    }
    else{
       
    }
    
    if (entryLocation == STATEMACHINE_IDLE) {
        // If we're waiting to be armed, don't run the flight control
        flightUpdate = 0;
    }

    if (flightUpdate) {
        flightUpdate = 0;
        //Input from Controller
        inputCapture();
        highLevelControl();
        lowLevelControl();
    }
    
    if(UPLINK_CHECK_FREQUENCY <= uplinkTimer){
        uplinkTimer = 0;
        readDatalink();
    }

    if(P0_SEND_FREQUENCY <= downlinkP0Timer){
        //debug("P0");
        //Compile and send data
        downlinkP0Timer = 0;
        writeDatalink(PRIORITY0);
    }
    else if(P1_SEND_FREQUENCY <= downlinkP1Timer){
        //debug("P1");
        //Compile and send data
        downlinkP1Timer = 0;
        writeDatalink(PRIORITY1);
    }
    else if(P2_SEND_FREQUENCY <= downlinkP2Timer || areGainsUpdated()){
        //debug("P2");
        //Compile and send data
        downlinkP2Timer = 0;
        writeDatalink(PRIORITY2);
    }
    else{
        //Then Sleep
    }
    //Loop it back again!
    inboundBufferMaintenance();
    outboundBufferMaintenance();
    asm("CLRWDT");
}

void forceStateMachineUpdate(){
    AMUpdate = 1;
}

void killPlane(char action){
    if (action){
        setProgramStatus(KILL_MODE);
    }
    else{
        setProgramStatus(MAIN_EXECUTION);
    }
}

