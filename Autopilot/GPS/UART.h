/**
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

#include <stdint.h>

/**
 * Status codes for whether to only enable TX, RX, or both on the specified interface
 */
#define UART_TX_ENABLE 0b01
#define UART_RX_ENABLE 0b10
#define UART_TX_RX_ENABLE 0b11

/**
 * Will initialize the specified UART interface for both RX and TX transmissions
 * at the specified baud rate.
 * @param interface Which interface to initialize (1 or 2?)
 * @param baudrate Baudrate to operate at for this interface
 * @param initial_buffer_size Initial sizes of the RX and TX buffers
 * @param max_buffer_size Max size of the RX and TX buffers
 * @param tx_rx specifies to turn on either TX only, RX only, or BOTH. See status codes above
 * 
 * Note that the initial_buffer_size and max_buffer_size should ideally be even 
 * multiples of each other. ie. 100, 200, or 400, 800, etc.
 * This is because of the underlying bytequeue implementation that doubles and halves
 * the queue, which works better with even multiples.
 */
void initUART(uint8_t interface, uint32_t baudrate, uint16_t initial_buffer_size, uint16_t max_buffer_size, uint8_t tx_rx);

/**
 * Read a byte from the uart RX buffer
 * @param interface The interface to read from (1 or 2)
 */
uint8_t readRXData(uint8_t interface);

/**
 * Send UART data synchronously to an interface
 * @param interface Which UART interface to send it through
 * @param data An array of bytes/chars to send
 * @param length The length of the aforementioned array of bytes/chars to send
 */
void sendTXData(uint8_t interface, uint8_t* data, uint16_t data_length);

/**
 * Get the current size of the uart rx buffer for the specified interface
 * @param interface
 * @return Size of the buffer in bytes
 */
uint16_t getRXSize(uint8_t interface);

#endif