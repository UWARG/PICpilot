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

#include <stdint.h>
#include <stdbool.h>

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
bool queueDownlinkPacket(unsigned char* data, unsigned int data_length);

/**
 * Sends the next queued up down the data link. Note that this will at most only 
 * send 1 packet down
 * @return true if something was sent down, false otherwise. This may be false if there is no
 * more data to send, or if the UART buffers are currently too full
 */
bool sendQueuedDownlinkPacket();

/**
 * Parses a received uplink packet received from the radio
 * @param length Length of the returned array, if applicable
 * @return A pointer to the parsed array of data, if it exists. This array will be
 *      created dynamically, so free it afterwords. This may also be NULL. A null 
 *      would indicate that 1) The UART RX buffer hasn't filled and the radio hasn't 
 *      finished its RX tranmission, 2) The parsed uplink was in some way invalid, 
 *      3) The packet is not applicable to received data from the link (ie. AT command)
 */
uint8_t* parseUplinkPacket(uint16_t* length);

#endif

