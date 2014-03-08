/**
 * net_inbound.c
 */

#include "net.h"
#include <stdlib.h>
#include "p33FJ256GP710.h"

#define START_DELIMITER 0x7E

typedef enum {
    EMPTY = 0,  // Can write new data here
    BUSY = 1,   // Data being written here
    READY = 2   // Data ready
} PacketStatus;

struct command* inBuffer[INBOUND_QUEUE_SIZE];
int inbuff_start = 0;
int inbuff_end = 0;

char rawPackets[RAW_PACKET_BUFFER_SIZE][128];
PacketStatus rawPacketStatus[RAW_PACKET_BUFFER_SIZE];

static int packetPos = 0;
static int payloadPos = 0;
static int payloadLength[RAW_PACKET_BUFFER_SIZE];

// Clean up the command
void destroyCommand( struct command* cmd ) {
    free( cmd );
    cmd = 0;
}

// Get the
struct command* popCommand() {
    struct command* cmd = inBuffer[inbuff_start];
    if ( ! cmd ) return 0;
    inBuffer[inbuff_start] = 0;
    inbuff_start = ( inbuff_start + 1 ) % INBOUND_QUEUE_SIZE;
    return cmd;
}

int pushCommand(struct command* cmd) {
    if ( inbuff_end == ( inbuff_start - 1 + INBOUND_QUEUE_SIZE ) % INBOUND_QUEUE_SIZE ) {
        return 0;   // If no room for commands, fail
    }
    inBuffer[inbuff_end] = cmd;     // Insert cmd at end of queue
    inbuff_end = ( inbuff_end + 1 ) % INBOUND_QUEUE_SIZE; // increment handling wrap
    return 1;
}

struct command* createCommand( char* rawPacket ) {
    struct command* cmd = malloc(sizeof(struct command));
    if ( ! cmd ) {  // Malloc failed ?
        return 0;
    }
    int k;
    char temp[128];
    for (k = 0; k < 128; k++) temp[k] = rawPacket[k];
    cmd->data_length = rawPacket[2];
    cmd->type = rawPacket[15];
    int i;
    int j = 0;
    for ( i = 16; i < 15 + cmd->data_length - 12; i++ ) {   // Received packet payload starts at 15 and has 12 bytes before len
        cmd->data[j++] = rawPacket[i];
    }
    cmd->data[j] = '\0';    // Null terminate the string so can use SendUart
    if ( cmd->data[0] == 0 ) {
        int zero = 1;
        zero += 1;
        return 0;
    }
    return cmd;
}

// Return 1 if packet is valid
int checkPacket( char* rawPacket) {
    return 1;                       // TODO: checksums are for suckers, because fuck you, thats why
}

void inboundBufferMaintenance(void) {
    int i;
    for ( i = 0; i < RAW_PACKET_BUFFER_SIZE; i++ ) {
        if ( rawPacketStatus[i] == READY && checkPacket(rawPackets[i]) ) {
            struct command* cmd = createCommand( rawPackets[i] );
            if ( cmd ) {            // create command was successful ?
                pushCommand( cmd ); // queue it up
                rawPacketStatus[i] = EMPTY;         // buffer is now good for writing another packet
            }
        }
    }
    if ( rawPacketStatus[0] == EMPTY ) {
        rawPacketStatus[0] = BUSY;
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
    unsigned char data = U2RXREG;
    if ( rawPacketStatus[packetPos] != BUSY ) {    // no buffer available to write
        IFS1bits.U2RXIF = 0;
        return;
    }
    switch ( payloadPos ) {
        case 0:
            if ( data != START_DELIMITER ) return;
            break;
        case 1:
            if ( data != 0 ) {
                payloadPos = 0;
                return;                 // packet length < 100 bytes, so msb == 0
            }
            break;
        case 2:
            payloadLength[packetPos] = data;
            break;
        default:        // Normally, don't do anything special
            break;
    }
    rawPackets[packetPos][payloadPos++] = data;
    if ( payloadPos && payloadPos == payloadLength[packetPos] + 3 + 1 ) {   // at end of packet
        rawPacketStatus[packetPos] = READY;
        payloadPos = 0;
        packetPos = ( packetPos + 1  ) % RAW_PACKET_BUFFER_SIZE;
        if ( rawPacketStatus[packetPos] == EMPTY ) {
            rawPacketStatus[packetPos] = BUSY;
        }
    }
    IFS1bits.U2RXIF = 0;
}