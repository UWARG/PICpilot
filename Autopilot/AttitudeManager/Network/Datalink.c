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

//to remove
#include <stdio.h>
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
        command->data = received + 1; //dont include the cmd id in the data
        command->next = NULL;
        
        //append the command to our queue
        if (datalinkCommandQueue.tail != NULL){
            datalinkCommandQueue.tail->next = command;
        } else { //if the tail doesnt exist then the head shouldnt either
            datalinkCommandQueue.head = command;
            datalinkCommandQueue.tail = command;
        }
    }
}

DatalinkCommand* popDatalinkCommand() {
    if (datalinkCommandQueue.head == NULL){
        return NULL;
    } else {
         DatalinkCommand* current = datalinkCommandQueue.head;
         if (current->next == NULL){
             datalinkCommandQueue.tail = NULL;
         }
         datalinkCommandQueue.head = current->next;
         current->next = NULL; //so that we cant use the list outside of this class
         return current;
    }
}


void freeDatalinkCommand(DatalinkCommand* to_destroy){
    free(to_destroy->data - 1); //since data represents the actual data array not including the cmd id
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
    return queueDownlinkPacket((uint8_t*)(&(telem->data)), sizeof(telem->data));
}