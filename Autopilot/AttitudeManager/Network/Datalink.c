/**
 * @file Datalink.c
 * @author Serj Babayan
 * @date March 12, 2017
 * @copyright Waterloo Aerial Robotics Group 2016-2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "Datalink.h"
#include "../Drivers/Radio.h"
#include "../../Common/Utilities/ByteQueue.h"
#include "../../Common/Utilities/Logger.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/** Index position in the DEFAULT_PACKET_ORDER array */
static uint8_t continuous_packet_order_index = 0;
static ByteQueue requested_packet_type_queue; //used to store the intermittent packets

static void pushDatalinkCommand(DatalinkCommand* command);

struct DatalinkCommandQueue {
    DatalinkCommand* head;
    DatalinkCommand* tail;
} datalinkCommandQueue;

void initDatalink(void){
    initRadio();
    datalinkCommandQueue.tail = NULL;
    datalinkCommandQueue.head = NULL;

    //allocate 16 bytes for the special packet type requests. Should be more than sufficient
    initBQueue(&requested_packet_type_queue, 16, 16);

    info("Datalink Initialized");
    //print out some debug info. Useful for debugging datalink
    debugInt("Position Block Size", sizeof(struct packet_type_position_block));
    debugInt("Status Block Size", sizeof(struct packet_type_status_block));
    debugInt("Gains Block Size", sizeof(struct packet_type_gain_block));
    debugInt("Channels Block Size", sizeof(struct packet_type_channels_block));
    debugInt("Telemetry Block Size", sizeof(TelemetryBlock));
}

void parseDatalinkBuffer(void) {
    uint16_t length;
    
    uint8_t* received = parseUplinkPacket(&length);
    
    //if we received a packet from the radio
    if (received != NULL){
        DatalinkCommand* command = malloc(sizeof(DatalinkCommand));
        
        if (command == NULL){ //we couldn't do malloc, so we'll discard of the data
            free(received);
            return;
        }
        
        command->data_length = length - 1; //data length doesnt acount the cmd id
        command->cmd = received[0];
        
        command->data = received;
        
        //don't include the command id in the data part of the command. This is required so
        //that the data that we later parse is word aligned, which is required for casting
        memcpy(command->data, command->data + 1, command->data_length);
        command->next = NULL;
        
        //append the command to our queue
        pushDatalinkCommand(command);
    }
}

DatalinkCommand* popDatalinkCommand(){
    DatalinkCommand* command = datalinkCommandQueue.head;
    
    if (command == datalinkCommandQueue.tail){
        datalinkCommandQueue.tail = NULL;
        datalinkCommandQueue.head = NULL;
    } else {
        datalinkCommandQueue.head = command->next;
    }
    return command;
}

void freeDatalinkCommand(DatalinkCommand* to_destroy){
    free(to_destroy->data);
    free(to_destroy);
}

bool queueTelemetryBlock(TelemetryBlock* telem) {
    return queueDownlinkPacket((uint8_t*)(telem), sizeof(TelemetryBlock));
}

PacketType getNextPacketType(){
    if (getBQueueSize(&requested_packet_type_queue) != 0){ //if the queue isnt empty
        return popBQueue(&requested_packet_type_queue);
    }

    uint8_t to_return = DEFAULT_PACKET_ORDER[continuous_packet_order_index];
    continuous_packet_order_index = (continuous_packet_order_index + 1)%sizeof(DEFAULT_PACKET_ORDER);
    return to_return;
}

void queuePacketType(PacketType type){
    pushBQueue(&requested_packet_type_queue, type);
}

/**
 * Adds a command to the command queue
 * @param command
 */
static void pushDatalinkCommand(DatalinkCommand* command)
{
    if (command == NULL){
        return;
    }
    
    command->next = NULL;
    
    if (datalinkCommandQueue.head == NULL){
        datalinkCommandQueue.head = command;
        datalinkCommandQueue.tail = command;
    } else {
        datalinkCommandQueue.tail->next = command;
        datalinkCommandQueue.tail = command;
    }
}

