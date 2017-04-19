/*
 * File:   StateMachine.c
 * Author: Chris Hajduk
 *
 * Created on June 9, 2015, 9:00 PM
 */

#include "StateMachine.h"
#include "./Network/Datalink.h"
#include "./AttitudeManager.h"
#include "../Common/Utilities/Logger.h"
#include "../Common/Common.h"
#include "main.h"
#include "../Common/Interfaces/InterchipDMA.h"
#include "Drivers/Radio.h"
#include "ProgramStatus.h"
#include "../Common/Clock/Timer.h"

//State Machine Triggers (Mostly Timers)
static int dmaTimer = 0;
static int uplinkTimer = 0;
static int downlinkPositionTimer = 0;
static int downlinkStatusTimer = 0;
static int downlinkChannelsTimer = 0;
static int imuTimer = 0;
static long int stateMachineTimer = 0;
static int dTime = 0;

static char AMUpdate = 0;
static char flightUpdate = 0;

void StateMachine(char entryLocation){
    dTime = (int)(getTime() - stateMachineTimer);
    stateMachineTimer += dTime;
    uplinkTimer += dTime;
    downlinkPositionTimer += dTime;
    downlinkStatusTimer += dTime;
    downlinkChannelsTimer += dTime;
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
    else if(newInterchipData() && checkDMA()){
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

    if(POSITION_SEND_FREQUENCY <= downlinkPositionTimer){
        downlinkPositionTimer = 0;
        writeDatalink(PACKET_TYPE_POSITION);
    }
    else if(STATUS_SEND_FREQUENCY <= downlinkStatusTimer){
        downlinkStatusTimer = 0;
        writeDatalink(PACKET_TYPE_STATUS);
    }
    else if(CHANNELS_SEND_FREQUENCY <= downlinkChannelsTimer){
        downlinkChannelsTimer = 0;
        writeDatalink(PACKET_TYPE_CHANNELS);
    } else if (areGainsUpdated() || showGains()){
        writeDatalink(PACKET_TYPE_GAINS);
    }
    
    parseDatalinkBuffer(); //read any incoming data from the Xbee and put in buffer
    sendQueuedDownlinkPacket(); //send any outgoing info
    
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
