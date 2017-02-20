/* 
 * @file UART.h
 * @author Serge Babayan
 * @date February 14, 2017, 9:05 PM
 * @brief Contains implementation for using the UART functionality of the chip (for both
 * interfaces)
 * @copyright Waterloo Aerial Robotics Group 2016 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef UART_H
#define	UART_H

/**
 * Defines which UART interfaces to enable/disable. Disabling means that the
 * interrupt cant be initialized, thus interrupts for it will not execute
 */
#define INTERFACE_UART1_ENABLED 0
#define INTERFACE_UART2_ENABLED 1

/**
 * Will initialize the specified UART interface for both RX and TX transmissions
 * at the specified baud rate.
 * @param interface Which interface to initialize (1 or 2?)
 * @param baudrate Baudrate to operate at for this interface
 */
void initUART(unsigned char interface, unsigned long int baudrate);

/**
 * This functions returns an array of the thus far received data. Note that the
 * returned data must be deallocated after it is read/used
 * @param interface The interface to read from (1 or 2)
 * @param length This variable will be set with the length of the returned RX data.
 *          Consider it the second return type of this function 
 * @return A character array representing the data thus far received from the UART interface
 */
unsigned char* getRXData(unsigned char interface, unsigned int* length);

/**
 * Queues data to send through UART. This will not necessarily send the data
 * immediately, however it will do so as fast as possible (as soon as the TX buffers
 * are made available). Therefore no need to poll for anything for sending data.
 * @param interface Which UART interface to send it through
 * @param data An array of bytes/chars to send
 * @param length The length of the aforementioned array of bytes/chars to send
 */
void quoueTXData(unsigned char interface, unsigned char* data, unsigned int length);

#endif

