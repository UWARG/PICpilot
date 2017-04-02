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
#include "../../Common/Utilities/Logger.h"
#include "../../Common/Interfaces/UART.h"
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#if USE_RADIO == RADIO_XBEE

/**
 * Set the internal buffer sizes of the TX and RX of the XBEE
 * Maximum length of an xbee transmission is 100 bytes, however the API command to do so has a 13 byte
 * overhead. So we'll need 113 bytes, but allocate 128 bytes for good measure
 */
#define XBEE_TX_BUFFER_LENGTH 128
#define XBEE_RX_BUFFER_LENGTH  128

/**
 * RSSI of the last received packet
 */
static uint8_t latest_rssi;

/**
 * Latest number of transmission errors
 */
static uint16_t transmission_errors;

/**
 * latest number of received errors
 */
static uint16_t received_error_count;

/**
 * The address that we'll send the packets to. This is the destination
 * address stored on the xbee configured via XCTU
 */
static uint64_t receiver_address;

/**
 * Current frame id of the response. Used to identify responses for AT commands
 */
static uint16_t current_frame_id;

/**
 * When we receive an AT response with this frame id, we know that we received
 * the last frame representing the radio status. Thus we know we can send over
 * the updated radio status (includes RSSI, transmission errors, and received error count
 */
static uint16_t transmit_status_frame_id;

/**
 * Representation of an xbee api frame, which is what we'll send to the xbee. Is a
 * node that contains a pointer to the next node in the queue, thus its a linked list
 */
typedef struct _XbeeApiFrame {
    /**
     * Frame time of the api frame. ie AT command, TX request
     */
    uint8_t frame_type;
    /**
     * This should be the payload data coming after the frame type and before the checksum.
     * This array should be dynamically allocated
     */
    uint8_t* data;

    /**
     * This should be the length of the payload described above
     */
    uint16_t data_length;
    struct _XbeeApiFrame* next_frame;
} XbeeApiFrame;

/**
 * Single object that represents the actual queue of data that we'll send over
 */
struct _XbeeFrameQueue {
    XbeeApiFrame* head;
    XbeeApiFrame* tail;
} XbeeFrameQueue;

static bool queueATCommand(char* at_command_id);
static void queueApiFrame(XbeeApiFrame* frame);
static uint8_t* parseReceivedApiFrame(uint8_t* data, uint16_t data_length, uint16_t* length);
static void parseReceivedATResponse(uint8_t* data, uint16_t data_length);
static XbeeApiFrame* popApiFrame(void);

void initRadio()
{
    //set it to the broadcast address for now
    receiver_address = XBEE_BROADCAST_ADDRESS;
    transmission_errors = 0;
    received_error_count = 0;
    latest_rssi = 0;
    current_frame_id = 1; //frame id's should start at 1

    XbeeFrameQueue.head = NULL;
    XbeeFrameQueue.tail = NULL;
    
    initUART(XBEE_UART_INTERFACE, XBEE_UART_BAUD_RATE, XBEE_UART_BUFFER_INITIAL_SIZE, XBEE_UART_BUFFER_MAX_SIZE, UART_TX_RX_ENABLE);

    //request destination address
    queueATCommand(XBEE_AT_COMMAND_DESTINATION_ADDRESS_HIGH);
    queueATCommand(XBEE_AT_COMMAND_DESTINATION_ADDRESS_LOW);
    info("Xbee Radio Initialized");
}

uint8_t getRadioRSSI()
{
    return latest_rssi;
}

uint16_t getRadioTransmissionErrors()
{
    return transmission_errors;
}

uint16_t getRadioReceiveErrors()
{
    return received_error_count;
}

void queueRadioStatusPacket()
{
    //read request for the parameters we care about
    queueATCommand(XBEE_AT_COMMAND_RSSI);
    queueATCommand(XBEE_AT_COMMAND_RECEIVED_ERROR_COUNT);
    queueATCommand(XBEE_AT_COMMAND_TRANSMISSION_ERRORS);
    transmit_status_frame_id = current_frame_id;
}

void clearRadioDownlinkQueue()
{
    XbeeApiFrame* packet = popApiFrame();
    while (packet != NULL) {
        packet = popApiFrame();
    }
    current_frame_id = 1; //also reset the current frame id
}

bool sendQueuedDownlinkPacket()
{
    //to temporarily store the api messages to send over the uart line
    static uint8_t send_buffer[XBEE_TX_BUFFER_LENGTH];

    send_buffer[0] = XBEE_START_DELIMITER;
    XbeeApiFrame* to_send = XbeeFrameQueue.head; //to save on typing. We're only peeking at the value here, not popping!

    if (to_send != NULL) {
        //the 5 comes from the start delimiter, 2 bytes for length, 1 for frame type, 1 for checksum
        uint16_t total_frame_length = to_send->data_length + 5;

        //the data_length doesn't include the frame_type byte
        uint16_t total_payload_length = to_send->data_length + 1;

        //if we cant transmit the entire API frame within our UART buffer, don't send anything (yet)
        if (getTXSpace(XBEE_UART_INTERFACE) < total_frame_length) {
            return false;
        }
        //weve got enough space for the packet. Pop it from the queue
        popApiFrame();
        
        send_buffer[1] = total_payload_length >> 8; //upper 8 bits of the length
        send_buffer[2] = total_payload_length & 0xFF; //lower 8 bits of the length
        send_buffer[3] = to_send->frame_type;

        uint16_t i;
        for (i = 0; i < to_send->data_length; i++) {
            send_buffer[i + 4] = to_send->data[i];
        }
        //add the checksum
        send_buffer[total_frame_length - 1] = total_payload_length % 0xFF;

        queueTXData(XBEE_UART_INTERFACE, send_buffer, total_frame_length); //queue the data for tranmission over UART

        XbeeFrameQueue.head = to_send->next_frame;
        free(to_send->data);
        free(to_send);
        return true;
    }
    return false;
}

bool queueDownlinkPacket(uint8_t* data, uint16_t data_length)
{
    int i;
    XbeeApiFrame* to_send = malloc(sizeof(XbeeApiFrame));

    if (to_send == NULL) {
        return false;
    }

    //a non explicit TX frame requires 13 more bytes of header data to be attached than the payload we're actually transmitted
    to_send->data = malloc(data_length + 13);

    if (to_send->data == NULL) {
        free(to_send);
        return false;
    }

    //this will be a TX api frame
    to_send->frame_type = XBEE_FRAME_TYPE_TX_REQUEST;

    //Frame id. Since there's not much benefit in processing response frames to check status of transmits, we'll set this to 0
    to_send->data[0] = 0;

    //copy the destination address to the tx frame
    for (i = 7; i >= 0; i--) {
        to_send->data[1 + (7 - i)] = (receiver_address >> i * 8) & 0xFF;
    }

    //reserved values
    to_send->data[9] = 0xFF;
    to_send->data[10] = 0xFE;

    //set the broadcast radius of this TX frame
    to_send->data[11] = XBEE_BROADCAST_RADIUS;

    //transmit options. 0x00 will use the saved transmit options parameter (the one saved in XCTU)
    to_send->data[12] = 0x00;

    //finally copy the payload data
    for (i = 0; i < data_length; i++) {
        to_send->data[i + 13] = data[i];
    }

    to_send->data_length = data_length + 13;

    to_send->next_frame = NULL;

    //queue the frame for transmission
    queueApiFrame(to_send);

    return true;
}

/**
 * This function will queue and read characters received in the UART byte buffer
 * until one of two things happen
 * 1) The buffer becomes empty
 * 2) The start delimiter is detected, in which this function will keep parsing
 * 3) If the start delimiter is detected, and the length of the packet is reached,
 *      will perform the checksum check on the received api frame. If it passses,
 *      will send the api frame over for parsing
 * @param length
 * @return A link to a dynamically allocated array representing data received from the groundstation,
 *      if applicable. NULL if the recieved packet was an AT command
 */
uint8_t* parseUplinkPacket(uint16_t* length)
{
    ///whether we're in the middle of parsing a received packet
    static bool parsing_rx_packet = false;

    //used to temporarily store the data as we're receiving it
    static uint8_t receive_buffer[XBEE_RX_BUFFER_LENGTH];

    //if we're currently parsing the packet, this indicates what position in the buffer
    static uint16_t rx_packet_pos;

    //the parsed length of the packet we're parsing. Used to tell when the packet is complete
    static uint16_t rx_packet_length;

    uint8_t byte; //bytes after the start delimiter

    //if we're parsing an existing packet. This block will wait for the start delimiter to be received
    if (parsing_rx_packet == false) {
        if (getRXSize(XBEE_UART_INTERFACE) == 0) {
            return NULL;
        }

        byte = readRXData(XBEE_UART_INTERFACE);

        //wait until we get the start delimiter
        while (byte != XBEE_START_DELIMITER) {
            if (getRXSize(XBEE_UART_INTERFACE) != 0) {
                byte = readRXData(XBEE_UART_INTERFACE);
            } else {
                return NULL;
            }
        }

        //here we have the start delimeter. subsequent packets are now part of a single API frame
        parsing_rx_packet = true;
        rx_packet_pos = 1;
        rx_packet_length = 0;
    }

    if (parsing_rx_packet == true) {
        while (getRXSize(XBEE_UART_INTERFACE) != 0) {
            byte = readRXData(XBEE_UART_INTERFACE);

            if (rx_packet_pos == 1) {
                //first byte is MSB of length of upcoming packet
                rx_packet_length = ((uint16_t) byte) << 8;
            } else if (rx_packet_pos == 2) {
                //second byte is the LSB of the length of the upcoming packet
                rx_packet_length += (uint16_t) byte;
            } else if (rx_packet_pos >= 3) {
                //copy over the payload (not including checksum). 2 because of the 2 length bytes
                if (rx_packet_pos <= rx_packet_length + 2) {
                    receive_buffer[rx_packet_pos - 3] = byte;
                } else { //we've finished copying the payload, check the checksum byte
                    if (byte % 0xFF != rx_packet_length % 0xFF) {
                        parsing_rx_packet = false;
                        return NULL;
                    } else {
                        parsing_rx_packet = false;
                        return parseReceivedApiFrame(receive_buffer, rx_packet_length, length);
                    }
                }
            }
            rx_packet_pos++;
        }
    }
    return NULL;
}

/**
 * Trigger a read of an at command
 * @param at_command_id 2 character id of the at command
 */
static bool queueATCommand(char* at_command_id)
{
    XbeeApiFrame* to_send = malloc(sizeof(XbeeApiFrame));

    if (to_send == NULL) {
        return false;
    }

    //an AT command request will take 3 bytes after the frame type dedicated
    to_send->data = malloc(3);

    if (to_send->data == NULL) {
        free(to_send);
        return false;
    }

    to_send->frame_type = XBEE_FRAME_TYPE_AT_COMMAND;
    to_send->data_length = 3;

    //set the frame id so that we get a response with the data
    to_send->data[0] = current_frame_id;
    current_frame_id++;

    //the 2 character code of the at command we're sending
    to_send->data[1] = at_command_id[0];
    to_send->data[2] = at_command_id[1];

    queueApiFrame(to_send);
    return true;
}

/**
 * Parses the payload of a received API frame. If it is a is a RX response, this
 * method will copy over the received data into a dynamic array and return
 * a pointer to it. Otherwise if its an AT response or anything else, will return
 * NULL and perform necessary actions if applicable
 * @param data Frame specific data. Comes after the length
 * @param data_length length of the frame specific data, not including the checksum byte
 * @param length The length of the payload if this was a RX frame
 * @return An array representing received TX data, or NULL otherwise
 */
static uint8_t* parseReceivedApiFrame(uint8_t* data, uint16_t data_length, uint16_t* length)
{
    uint8_t frame_type = data[0];

    switch (frame_type) {
    case XBEE_FRAME_TYPE_AT_RESPONSE:
        parseReceivedATResponse(data, data_length);
        return NULL;
    case XBEE_FRAME_TYPE_RX_INDICATOR:
        //From the xbee api docs, we not care about the sender address in our case or what the receive options are
        //so we'll just skip to the payload ,since thats the only thing we really care about
        //according to the docs, the actual RF packet will start at 13th byte of the api frame payload
        data_length = data_length - 12;
        uint8_t* copied = malloc(data_length);

        //if we couldnt allocate the data, discard the packet
        if (copied == NULL) {
            return NULL;
        }

        uint16_t i;
        for (i = 0; i < data_length; i++) {
            copied[i] = data[i + 12];
        }

        *length = data_length;
        return copied;
    default:
        return NULL;
    }
}

/**
 * This function parses a received AT response packet. Based on the contents,
 * modifies certain parameters of the driver. For example, if an AT packet containing
 * a source address comes in, will flip the transmissions flag
 * @param data Frame specific data (comes after length)
 * @param data_length Length of frame specific data, not including the checksum
 */
static void parseReceivedATResponse(uint8_t* data, uint16_t data_length)
{
    //a valid response frame payload needs to have a data length of at least 5, not including the checksum
    if (data_length < 5) {
        return;
    }

    //frame type is at 0, which we dont care about here
    uint8_t frame_id = data[1];
    char at_command[3];

    at_command[0] = data[2];
    at_command[1] = data[3];
    at_command[2] = 0;

    //5th byte is the command status.
    if (data[4] == XBEE_AT_COMMAND_STATUS_OK) {
        if (strcmp(at_command, XBEE_AT_COMMAND_RSSI) == 0) {
            //the RSSI is just a byte
            if (data_length < 6) {
                return;
            }
            latest_rssi = data[5];
        } else if (strcmp(at_command, XBEE_AT_COMMAND_RECEIVED_ERROR_COUNT) == 0) {
            if (data_length < 7) {
                return;
            }
            received_error_count = 0;
            received_error_count = (uint16_t) data[5] << 8;
            received_error_count += data[6];
        } else if (strcmp(at_command, XBEE_AT_COMMAND_TRANSMISSION_ERRORS) == 0) {
            if (data_length < 7) {
                return;
            }
            transmission_errors = 0;
            transmission_errors = (uint16_t) data[5] << 8;
            transmission_errors += data[6];
        } else if (strcmp(at_command, XBEE_AT_COMMAND_DESTINATION_ADDRESS_HIGH) == 0) {
            //command data should be 8 bytes. Thus total data length should be 13
            if (data_length < 9) {
                return;
            }

            receiver_address = receiver_address & 0xFFFFFFFF; //clear upper 32 bits
            receiver_address += (uint64_t) data[5] << 56;
            receiver_address += (uint64_t) data[6] << 48;
            receiver_address += (uint64_t) data[7] << 40;
            receiver_address += (uint64_t) data[8] << 32;
        } else if (strcmp(at_command, XBEE_AT_COMMAND_DESTINATION_ADDRESS_LOW) == 0) {
            //command data should be 8 bytes. Thus total data length should be 13
            if (data_length < 9) {
                return;
            }

            receiver_address = receiver_address & 0xFFFFFFFF00000000; //clear lower 32 bits
            receiver_address += (uint64_t) data[5] << 24;
            receiver_address += (uint64_t) data[6] << 16;
            receiver_address += (uint64_t) data[7] << 8;
            receiver_address += (uint64_t) data[8];
        }
    }

    if (frame_id == transmit_status_frame_id) {
        //TODO: here we would build the mavlink readio status packet and send it over
    }
}

/**
 * Pops an element from the xbee frame queue
 * @return Null if nothing to pop
 */
static XbeeApiFrame* popApiFrame(){
    XbeeApiFrame* packet = XbeeFrameQueue.head;
    
    if (packet == XbeeFrameQueue.tail){
        XbeeFrameQueue.tail = NULL;
        XbeeFrameQueue.head = NULL;
    } else {
        XbeeFrameQueue.head = packet->next_frame;
    }
    return packet;
}

/**
 * Queues an API frame for transmission, however does not send it yet
 * @param frame
 */
static void queueApiFrame(XbeeApiFrame* frame)
{
    if (frame == NULL){
        return;
    }
    
    frame->next_frame = NULL;
    
    if (XbeeFrameQueue.head == NULL){
        XbeeFrameQueue.head = frame;
        XbeeFrameQueue.tail = frame;
    } else {
        XbeeFrameQueue.tail->next_frame = frame;
        XbeeFrameQueue.tail = frame;
    }
}

#endif
