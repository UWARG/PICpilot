/* 
 * File:   Attitude_Simulator.h
 * Author: WARG
 *
 * Created on April 13, 2016, 12:57 AM
 */

#ifndef ATTITUDE_SIMULATOR_H
#define	ATTITUDE_SIMULATOR_H
#include "../Common/UART1.h"
/*
 * File:   UART.h
 * Author: Mitch
 *
 * Created on June 15, 2013, 2:52 PM
 */
#define RAW_PACKET_BUFFER_SIZE 32
#define SEND_PACKET_SIZE (4*sizeof(int) + 2)
#define START_DELIMITER 0xAA
#define END_DELIMITER 0xBB

void setSimData(void);//struct ImuData *input);

struct ImuData *getSimData();

void setTxPacket(char, int, int, int, int);

char getRxPacketStatus();

struct ImuData {
    unsigned char start;
    unsigned char pad1[3];
    float rollAngleRate;
    float pitchAngleRate;
    float yawAngleRate;

    float rollAngle;
    float pitchAngle;
    float yawAngle;

    unsigned char end;
    unsigned char pad2[3];
    // TODO: Add additional telemetry to be sent here
};

union byteint
{
    char b[sizeof(int)];
    int i;
};

#endif	/* UART_H */