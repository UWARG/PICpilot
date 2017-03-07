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
 * 64-bit address indicating which address this xbee should broadcast to. Should
 * be the same as the source address of the xbee on the ground side
 */
#define XBEE_DESTINATION_ADDRESS 0x0013A20040B47E6B

/**
 * Sets maximum number of hops a broadcast transmission can occur. If set to 0, 
 * the broadcast radius will be set to the maximum hops value
 */
#define XBEE_BROADCAST_RADIUS 0x00

#endif

