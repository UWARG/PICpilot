/**
 * @file UART.h
 * @author Serge Babayan
 * @date February 14, 2017, 9:05 PM
 * @brief Contains implementation for using the UART functionality of the chip
 * 
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef UART_H
#define	UART_H

#include <stdint.h>

/**
 * Will initialize the specified UART interface for both RX and TX transmissions
 * at the specified baud rate.
 * @param interface Which interface to initialize (1 or 2?)
 * @param baudrate Baudrate to operate at for this interface
 */
void initUART(uint8_t interface, uint32_t baudrate);

/**
 * Send UART data synchronously to an interface
 * @param interface Which UART interface to send it through
 * @param data An array of bytes/chars to send
 * @param length The length of the aforementioned array of bytes/chars to send
 */
void sendTXData(uint8_t interface, uint8_t* data, uint16_t data_length);

#endif