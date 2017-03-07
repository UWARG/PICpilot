/**
 * @file Radio.h
 * @author Serge Babayan
 * @date March 6, 2017
 * 
 * @brief Contains the interface that all radio drivers must implement. Allow to send
 * data down the downlink, and parse uplink data.
 * 
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef RADIO_H
#define	RADIO_H

#define RADIO_XBEE 0
#define RADIO_3DR 1

/**
 * Which radio driver to compile for
 */
#define USE_RADIO RADIO_XBEE

/**
 * Initializes the selected radio driver
 */
void initRadio();

/**
 * Instructs the radio driver to retrieve information about the error counts and rssi
 * of the radio, and queue this data to be sent down the downlink. In the case of the
 * 3DR radio, this method will do nothing as the radio attaches this data automatically.
 * 
 * When this packet is sent down, the data relay should also recognize it and attach its
 * uplink rssi to the mavlink packet
 */
void queueRadioDownlinkPacket();

/**
 * Queues data to be sent down the data link
 * @param data Bytes of the payload data to send
 * @param data_length Length of the aforementioned data
 * @return 1 if the downlink data was successfully queued. 0 otherwise (probably because of malloc)
 */
char queueDownlinkPacket(unsigned char* data, unsigned int data_length);

/**
 * Sends the next queued up down the data link. Note that this will at most only 
 * send 1 packet down
 * @return 1 if something was sent down, 0 otherwise. This may be 0 if there is no
 * more data to send, or if the UART buffers are currently too full
 */
char sendQueuedDownlinkPacket();

#endif

