/* 
 * @file UART.h
 * @author Serge Babayan
 * @date February 14, 2017, 9:05 PM
 * @brief Contains implementation for using the UART functionality of the chip (for both
 * interfaces)
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef UART_H
#define	UART_H

/**
 * Initial sizes of the uart 1 and 2 tx and rx buffers in bytes. Note initial
 * as the buffer will resize if necessary
 */
#define INITIAL_UART1_TX_BUFFER_SIZE 100
#define INITIAL_UART1_RX_BUFFER_SIZE 100
#define INITIAL_UART2_TX_BUFFER_SIZE 100
#define INITIAL_UART2_RX_BUFFER_SIZE 100

/**
 * Will initialize the specified UART interface for both RX and TX transmissions
 * at the specified baud rate.
 * @param interface Which interface to initialize (1 or 2?)
 * @param baudrate Baudrate to operate at for this interface
 * @param initial_size Initial sizes of the RX and TX buffers
 */
void initUART(unsigned char interface, unsigned long int baudrate, unsigned int initial_size);

/**
 * Read a byte from the uart RX buffer
 * @param interface The interface to read from (1 or 2)
 */
unsigned char readRXData(unsigned char interface);

/**
 * Queues data to send through UART. This will not necessarily send the data
 * immediately, however it will do so as fast as possible (as soon as the TX buffers
 * are made available). Therefore no need to poll for anything for sending data.
 * @param interface Which UART interface to send it through
 * @param data An array of bytes/chars to send
 * @param length The length of the aforementioned array of bytes/chars to send
 */
void quoueTXData(unsigned char interface, unsigned char* data, unsigned int data_length);

#endif