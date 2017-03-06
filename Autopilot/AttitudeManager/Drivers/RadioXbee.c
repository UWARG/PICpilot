/**
 * @file RadioXbee.c
 * @author Serge Babayan
 * @date March 6, 2017
 * @see https://uwarg-docs.atlassian.net/wiki/display/PicPilot/Xbee+Telemetry+System
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "./Radio.h"
#include "./RadioXbee.h"

#if USE_RADIO == RADIO_XBEE

/**
 * This is the first byte of all Xbee packets
 */
#define XBEE_START_DELIMITER 0x7E

/**
 * API id to send an AT command to the xbee module
 */
#define XBEE_API_AT_COMMAND 0x08

/**
 * API id to tell the xbee to send a TX request with the attached payload
 */
#define XBEE_API_TX_REQUEST 0x10

/**
 * API id returned from the xbee as a response from an AT command
 */
#define XBEE_API_AT_RESPONSE 0x88

/**
 * API id from the xbee indicating the transmit status of the last packet
 */
#define XBEE_API_TRANSMIT_STATUS 0x8B

/**
 * API id from the xbee indicating data was received from the uplink
 */
#define XBEE_API_RX_INDICATOR 0x90

static char send_buffer[100];

static void prepareAPICommand(char length, char frame_type, char response_frame_id, ){
    
}


static char latest_rssi;
static char transmission_errors;
static char received_error_count;

static void sendATCommand(){
    send_buffer[0] = XBEE_START_DELIMITER;
    
}


void sendRadioStatusPacket(){
    
}

void sendTXData(){
    
}


static void initRadio(){
    
}



#endif

