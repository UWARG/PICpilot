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

struct DatalinkCommandQueue {
    DatalinkCommand* head;
    DatalinkCommand* tail;
} datalinkCommandQueue;

void initDatalink(void){
    initRadio();
}

void inboundBufferMaintenance(void) {
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
        command->data = &received[1];
        command->next = NULL;
        
        //append the command to our queue
        if (datalinkCommandQueue.tail != NULL){
            datalinkCommandQueue.tail->next = command;
        } else {
            datalinkCommandQueue.head = command;
        }
    }
}

DatalinkCommand* popCommand() {
    if (datalinkCommandQueue.head == NULL){
        return NULL;
    } else {
         DatalinkCommand* current = datalinkCommandQueue.head;
         if (current->next == NULL){
             datalinkCommandQueue.tail = NULL;
         }
         datalinkCommandQueue.head = current->next;
         return current;
    }
}

TelemetryBlock* createTelemetryBlock(p_priority packet) {
    TelemetryBlock* telem = malloc(sizeof(TelemetryBlock));
    telem->type = (uint8_t)packet;
    return telem;
}

bool queueTelemetryBlock(TelemetryBlock* telem) {
    return queueDownlinkPacket((uint8_t*)telem->data, sizeof(telem->data));
}