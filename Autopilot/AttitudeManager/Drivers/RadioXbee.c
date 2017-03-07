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
#include "../../Common/Interfaces/UART.h"
#include <stddef.h>
#include <stdlib.h>

#if USE_RADIO == RADIO_XBEE

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
#define XBEE_FRAME_ID_RESPONSE 0x88

/**
 * API id from the xbee indicating the transmit status of the last packet
 */
#define XBEE_FRAME_ID_TRANSMIT_STATUS 0x8B

/**
 * API id from the xbee indicating data was received from the uplink
 */
#define XBEE_FRAME_ID_RX_INDICATOR 0x90

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
 * RSSI of the last received packet
 */
static unsigned char latest_rssi;

/**
 * Latest number of transmission errors
 */
static unsigned int transmission_errors;

/**
 * latest number of received errors
 */
static unsigned int received_error_count;

/**
 * Current frame id of the response. Used to identify responses for AT commands
 */
static unsigned int current_frame_id;

/**
 * A buffer for storing the API messages to send over to the Xbee. Maximum length
 * of an xbee transmission is 100 bytes, however the API command to do so has a 13 byte
 * overhead. So we'll need 113 bytes, but allocate 128 bytes for good measure
 */
static unsigned char send_buffer[128];

/**
 * Representation of an xbee api frame, which is what we'll send to the xbee. Is a
 * node that contains a pointer to the next node in the queue, thus its a linked list
 * 
 * Note that if issuing an api frame that expects a response, you will need to include
 * the frame id inside the data
 */
typedef struct _XbeeApiFrame{
    unsigned char* data;
    unsigned char frame_type;
    unsigned int data_length;
    struct _XbeeApiFrame* next_frame;
} XbeeApiFrame;

/**
 * Single object that represents the actual queue of data that we'll send over
 */
struct _XbeeFrameQueue{
    XbeeApiFrame* head;
    XbeeApiFrame* tail;
} XbeeFrameQueue;


static char  queueATCommand(char* at_command_id, char* optional_payload, unsigned int optional_payload_length);
static void queueApiFrame(XbeeApiFrame* frame);

void initRadio(){
    transmission_errors = 0;
    received_error_count = 0;
    latest_rssi = 0;
    
    XbeeFrameQueue.head = NULL;
    XbeeFrameQueue.tail = NULL;
}

void queueRadioDownlinkPacket(){
    //read request for the parameters we care about
    queueATCommand(XBEE_AT_COMMAND_RSSI, NULL, 0);
    queueATCommand(XBEE_AT_COMMAND_RECEIVED_ERROR_COUNT, NULL, 0);
    queueATCommand(XBEE_AT_COMMAND_TRANSMISSION_ERRORS, NULL, 0);
}

char sendQueuedDownlinkPacket(){
    send_buffer[0] = XBEE_START_DELIMITER;
    XbeeApiFrame* to_send = XbeeFrameQueue.head; //to save on typing
    
    if (to_send != NULL){
        //if we cant transmit the entire API frame within our UART buffer, don't send anything (yet)
        //the 5 comes from the start delimiter, 2 bytes for length, 1 for frame type, 1 for checksum
        if (to_send->data_length + 5 >= getTXSpace(XBEE_UART_INTERFACE)){
            return 0;
        }
        send_buffer[1] = (unsigned char)(to_send->data_length >> 8); //upper 8 bits of the length
        send_buffer[2] = (unsigned char)(to_send->data_length & 0x00FF); //lower 8 bits of the length
        send_buffer[3] = to_send->frame_type;
        unsigned int i;
        for (i = 0; i < to_send->data_length; i++){
            send_buffer[i + 4] = to_send->data[i];
        }
        send_buffer[to_send->data_length + 5] = to_send->data_length % 0xFF; //add the checksum
        
        queueTXData(XBEE_UART_INTERFACE, send_buffer, to_send->data_length + 5); //queue the data for tranmission over UART
        
        XbeeFrameQueue.head = to_send->next_frame;
        free(to_send->data);
        free(to_send);
        return 1;
    }
    return 0;
}

char queueDownlinkData(unsigned char* data, unsigned int data_length){
    int i;
    XbeeApiFrame* to_send = (XbeeApiFrame*)malloc(sizeof(XbeeApiFrame));
    
    if (to_send == NULL){
        return 0;
    }
    
    //a non explicit TX frame requires 13 more bytes of header data to be attached than the payload we're actually transmitted
    to_send->data = (unsigned char*)malloc(data_length + 13);
    
    if (to_send->data == NULL){
        return 0;
    }
    
    //this will be a TX api frame
    to_send->frame_type = XBEE_FRAME_TYPE_TX_REQUEST;
    
    //Frame id. Since there's not much benefit in processing response frames to check status of transmits, we'll set this to 0
    to_send->data[0] = 0;
    
    //copy the destination address to the tx frame
    for (i = 7; i >= 0; i--){
        to_send->data[8 - i] = (XBEE_DESTINATION_ADDRESS >> i*8) & 0xFF;
    }
    
    //reserved values
    to_send->data[9] = 0xFF;
    to_send->data[10] = 0xFE;
    
    //set the broadcast radius of this TX frame
    to_send->data[11] = XBEE_BROADCAST_RADIUS;
    
    //transmit options. 0x00 will use the saved transmit options parameter (the one saved in XCTU)
    to_send->data[12] = 0x00;
    
    //finally copy the payload data
    for(i=0; i < data_length; i++){
        to_send->data[i + 13] = data[i];
    }
    
    to_send->data_length = data_length + 13;
    
    to_send->next_frame = NULL;
    
    //queue the frame for transmission
    queueApiFrame(to_send);
    
    return 1;
}

/**
 * Queues an API frame for transmission, however does not send it yet
 * @param frame
 */
static void queueApiFrame(XbeeApiFrame* frame){
     //append this frame to the queue of frames to send
    if (XbeeFrameQueue.tail == NULL){
        XbeeFrameQueue.head = frame;
    } else {
        XbeeFrameQueue.tail->next_frame = frame;
    }
    XbeeFrameQueue.tail = frame;
}

/**
 * Trigger a read of an at command
 * @param at_command_id 2 character id of the at command
 * @param optional_payload An optional payload to write to the at command. If this value
 *      is NULL, the AT command will be a read and the response will be parsed. If not, the AT
 *      command will be a write and no response will be generated
 * @param optional_payload_length Should be the length of the optional payload, if applicable.
 *      If the optional_payload is NULL, it doesnt matter what this value is
 */
static char  queueATCommand(char* at_command_id, char* optional_payload, unsigned int optional_payload_length){
    static unsigned int i;
    XbeeApiFrame* to_send = (XbeeApiFrame*)malloc(sizeof(XbeeApiFrame));
    
    if (to_send == NULL){
        return 0;
    }
    
    if (optional_payload == NULL){
        optional_payload_length = 0;
    }
    
    //a read only AT command request will always have a payload of length 3
    to_send->data = (unsigned char*)malloc(3 + optional_payload_length);
    
    if (to_send->data == NULL){
        return 0;
    }
    
    to_send->frame_type = XBEE_FRAME_TYPE_AT_COMMAND;
    to_send->data_length = 3 + optional_payload_length;
    
    //the frame id. This should be non-zero and incrementing so that we get an at resopnse afterwords
    //we only want the at response for a read at command, not a write
    if (optional_payload == NULL){
        to_send->data[0] = current_frame_id;
        current_frame_id ++;
    } else{
        to_send->data[0] = 0;
    }
    
    //the 2 character code of the at command we're sending
    to_send->data[1] = at_command_id[0];
    to_send->data[2] = at_command_id[1];
    
    //copy the payload over, if it exists
    for(i = 0; i < optional_payload_length; i++){
        to_send->data[3 + i] = optional_payload[i];
    }
    
    queueApiFrame(to_send);
    return 1;
 }

#endif
