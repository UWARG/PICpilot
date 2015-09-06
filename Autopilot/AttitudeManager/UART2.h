/*
 * File:   UART2.h
 * Author: Mitch
 *
 * Created on June 15, 2013, 2:52 PM
 */

#ifndef UART2_H
#define	UART2_H
#include "main.h"

void InitUART2();
void UART2_SendString(char *s);
void UART2_SendChar(char data);

void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt( void );

#if PATH_MANAGER && !GPS_OLD

 typedef struct _UART_RX_Buffer {
     char data[256];
     short get;
     short put;
     //short size;
} UART_RX_Buffer;

void init_rx_buffer(UART_RX_Buffer *buff);
void write_rx_buffer(char c, UART_RX_Buffer *buff);
char read_rx_buffer(UART_RX_Buffer *buff);



#endif

#endif	/* UART2_H */
