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

//int getRxPacketStatus();

//void setTxPacket(int, int, int, int);
//
//struct ImuData {
//    unsigned char start;
//    unsigned char pad1[3];
//    float rollAngleRate;
//    float pitchAngleRate;
//    float yawAngleRate;
//
//    float rollAngle;
//    float pitchAngle;
//    float yawAngle;
//
//    unsigned char end;
//    unsigned char pad2[3];
//    // TODO: Add additional telemetry to be sent here
//};
//
//union byteint
//{
//    char b[sizeof(int)];
//    int i;
//};
//
//struct ImuData *getSimData();
//void setSimData(void);//struct ImuData *input);

#endif	/* UART_H */