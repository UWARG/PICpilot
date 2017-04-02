/**
 * @file RadioXbee.h
 * @author Serge Babayan
 * @date March 6, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef RADIOXBEE_H
#define	RADIOXBEE_H

/**
 * Which baud rate to communicate the xbee with
 */
#define XBEE_BAUD_RATE 115200

/**
 * Which UART interface the xbee should use
 */
#define XBEE_UART_INTERFACE 2

/**
 * Sets maximum number of hops a broadcast transmission can occur. If set to 0, 
 * the broadcast radius will be set to the maximum hops value
 */
#define XBEE_BROADCAST_RADIUS 0x00

/**
 * This is the 64-bit broadcast address that the Xbee will broadcast to by default
 */
#define XBEE_BROADCAST_ADDRESS ((uint64_t)0xFFFF)

/**
 * This is the first byte of all Xbee packets
 */
#define XBEE_START_DELIMITER 0x7E

/**
 * API id to send an AT command to the xbee module
 */
#define XBEE_FRAME_TYPE_AT_COMMAND 0x08

/**
 * API id to tell the xbee to send a TX request with the attached payload
 */
#define XBEE_FRAME_TYPE_TX_REQUEST 0x10

/**
 * API id returned from the xbee as a response from an AT command
 */
#define XBEE_FRAME_TYPE_AT_RESPONSE 0x88

/**
 * API id from the xbee indicating data was received from the uplink
 */
#define XBEE_FRAME_TYPE_RX_INDICATOR 0x90

/**
 * AT command code to retrieve the latest received signal strength
 */
#define XBEE_AT_COMMAND_RSSI "DB"

/**
 * AT command code to retrieve the current received error count
 */
#define XBEE_AT_COMMAND_RECEIVED_ERROR_COUNT "ER"

/**
 * AT command code to retrieve the transmission errors
 */
#define XBEE_AT_COMMAND_TRANSMISSION_ERRORS "TR"

/**
 * AT command code to retrieve the 64-bit destination address of the xbee
 */
#define XBEE_AT_COMMAND_DESTINATION_ADDRESS_HIGH "DH"
#define XBEE_AT_COMMAND_DESTINATION_ADDRESS_LOW "DL"

/**
 * Xbee AT command status codes. Indicates whether the payload of the AT response
 * is valid (ie. should we save the RSSI?)
 */
#define XBEE_AT_COMMAND_STATUS_OK 0
#define XBEE_AT_COMMAND_STATUS_ERROR 1

#endif

