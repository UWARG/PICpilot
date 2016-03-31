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

unsigned char* getData();

void setData(char* msg);

int getRxPacketStatus();

#endif	/* UART_H */