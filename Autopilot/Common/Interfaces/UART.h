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
void initUART(unsigned char interface, unsigned long int baudrate, unsigned int initial_buffer_size, unsigned int max_buffer_size, unsigned char tx_rx);

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
void queueTXData(unsigned char interface, unsigned char* data, unsigned int data_length);

/**
 * Get the number of bytes that are guaranteed to be succesfully queued to the TX buffer.
 * This is useful if sending partial amounts of data is unacceptable. You can use
 * this to cancel a UART send command if there's no pointing in sending the data (or poll
 * until this is full enough)
 * @param interface
 * @return number of bytes that can be written
 */
unsigned int getTXSpace(unsigned char interface);

#endif