/**
 * @file Datalink.c
 * @author Serj Babayan
 * @date March 12, 2017
 * @copyright Waterloo Aerial Robotics Group 2016-2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "Datalink.h"
#include "../Drivers/Radio.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

static void pushDatalinkCommand(DatalinkCommand* command);

struct DatalinkCommandQueue {
    DatalinkCommand* head;
    DatalinkCommand* tail;
} datalinkCommandQueue;

void initDatalink(void){
    initRadio();
    datalinkCommandQueue.tail = NULL;
    datalinkCommandQueue.head = NULL;
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

TelemetryBlock* createTelemetryBlock(p_priority packet) {
    TelemetryBlock* telem = malloc(sizeof(TelemetryBlock));
    
    if (telem != NULL){
        telem->type = (uint8_t)packet;
    }
 
    return telem;
}

bool queueTelemetryBlock(TelemetryBlock* telem) {
    return queueDownlinkPacket((uint8_t*)(telem), sizeof(TelemetryBlock));
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
