/*
 * File:   UART.h
 * Author: Mitch
 *
 * Created on June 15, 2013, 2:52 PM
 */

#ifndef UART_H
#define	UART_H

void InitUART1();

void UART1_SendChar(char data);

void UART1_SendString(char *s);

//unsigned char* getData();
//
//void setData(char* msg);

int getRxPacketStatus();

struct ImuData {
    char start;
    char pad1[3];
    float rollAngleRate;
    float pitchAngleRate;
    float yawAngleRate;

    float rollAngle; 
    float pitchAngle;
    float yawAngle;

    char end;
    char pad2[3];
    // TODO: Add additional telemetry to be sent here
};

struct ImuData *getSimData();
void setSimData(struct ImuData *input);

#endif	/* UART_H */