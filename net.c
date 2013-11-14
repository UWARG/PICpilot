// net.c
//
// ONLY EDIT THIS FILE IF YOU ARE REALLY SURE YOU KNOW WHAT YOU ARE
// DOING.
//
// This code contains lock free data structures and scenarios where
// their validity is really unstable.
// 
// Seriously, don't touch this code

#include "net.h"
#include <stdlib.h>
#include "p33FJ256GP710.h"
#include "UART2.h"

const unsigned int PACKET_LENGTH = API_HEADER_LENGTH + sizeof(struct telem_block) + 1;

struct telem_block *outBuffer [OUTBOUND_QUEUE_SIZE];
int outbuff_start = 0;
int outbuff_end = 0;

struct telem_block *inBuffer [INBOUND_QUEUE_SIZE];
int inbuff_start = 0;
int inbuff_end = 0;

struct telem_block *debugTelemetry;

struct telem_buffer stagingBuffer;

// Initialize the data link
int initDataLink(void) {
    InitUART2();
    IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupt
    stagingBuffer.header[0]='\x42';
    return 0;
}

// Create a telem block returns null if fails
struct telem_block *createTelemetryBlock(void) {
    struct telem_block *telem = malloc(sizeof (struct telem_block));
    return telem;
}

// Create a telemetry block to use for debugging, only creates one instance
struct telem_block *getDebugTelemetryBlock(void) {
    // If the telemetry block does not exist, create it filled with ones
    // of the respective types
    if (debugTelemetry == 0) {
        debugTelemetry = createTelemetryBlock();
        debugTelemetry->millis = (long long) 1;
        debugTelemetry->lat = (long double) 1;
        debugTelemetry->lon = (long double) 1;
        debugTelemetry->pitch = (float) 1;
        debugTelemetry->roll = (float) 1;;
        debugTelemetry->yaw = (float) 1;
        debugTelemetry->pitchRate = (float) 1;
        debugTelemetry->rollRate = (float) 1;
        debugTelemetry->yawRate = (float) 1;
        debugTelemetry->pitch_gain = (float) 1;
        debugTelemetry->roll_gain = (float) 1;
        debugTelemetry->yaw_gain = (float) 1;
        debugTelemetry->pitchSetpoint = (int) 1;
        debugTelemetry->rollSetpoint = (int) 1;
        debugTelemetry->yawSetpoint = (int) 1;
        debugTelemetry->editing_gain = 'X';
    }
    return debugTelemetry;
}

// Destroy a telemetryBlock
void destroyTelemetryBlock(struct telem_block *telem) {
    free(telem);
    telem = 0;
}

// Add a telem_block to the outbound telemetry queue
// Returns the position in the queue or -1 if no room in queue
int pushOutboundTelemetryQueue(struct telem_block *telem) {
    if (getOutboundQueueLength() >= OUTBOUND_QUEUE_SIZE) {
        return -1;
    }
    outBuffer[outbuff_end] = telem;
    outbuff_end++;
    outbuff_end = outbuff_end % OUTBOUND_QUEUE_SIZE;
    return getOutboundQueueLength();
}

// Get the number of items waiting to be sent
int getOutboundQueueLength(void) {
    int length = outbuff_end - outbuff_start;
    if ( length < 0 ) {
        return length + OUTBOUND_QUEUE_SIZE;
    } else {
        return length;
    }
}

// Clear all telem blocks waiting to be sent
int clearOutboundTelemetryQueue(void) {
    int cleared = 0;
    for (cleared = 0; cleared < OUTBOUND_QUEUE_SIZE; cleared++) {
        if (outBuffer[cleared] != 0) {
            destroyTelemetryBlock(outBuffer[cleared]);
            outBuffer[cleared] = 0;
        }
    }
    return cleared;
}

// Do buffer maintenance
void bufferMaintenance(void) {
    //UART1_SendChar('B');
    if ( stagingBuffer.sendIndex >= PACKET_LENGTH ) {
        destroyTelemetryBlock(stagingBuffer.telemetry.asStruct);
        if ( getOutboundQueueLength() ) {
            stageTelemetryBlock(popOutboundTelemetryQueue());
        }
    } else if ( stagingBuffer.telemetry.asStruct == 0 && getOutboundQueueLength() ) {
        stageTelemetryBlock(popOutboundTelemetryQueue());
    }
}

void sendNextByte(void) {
    unsigned char sendByte; // The byte to send
    if ( stagingBuffer.sendIndex < API_HEADER_LENGTH ) {
        //while (U2STAbits.TRMT == 0);
        sendByte = stagingBuffer.header[stagingBuffer.sendIndex] & 0xFF;
        // Compute checksum
        if (stagingBuffer.sendIndex >= 3) {
            stagingBuffer.checksum += sendByte & 0xFF;
        }
    } else if ( stagingBuffer.sendIndex < PACKET_LENGTH - 1 ) {
        sendByte = stagingBuffer.telemetry.asArray[stagingBuffer.sendIndex - API_HEADER_LENGTH] & 0xFF;
        stagingBuffer.checksum += sendByte & 0xFF;
    } else if ( stagingBuffer.sendIndex == PACKET_LENGTH - 1) {
        sendByte = 0xFF - (stagingBuffer.checksum & 0xFF);
    } else {
        IFS1bits.U2TXIF = 0;
        return;
    }

    stagingBuffer.sendIndex++;
    IFS1bits.U2TXIF = 0;
    U2TXREG = sendByte;
}

// Put the next telemetry
void stageTelemetryBlock(struct telem_block *telem) {
    stagingBuffer.telemetry.asStruct = telem;
    generateApiHeader(stagingBuffer.header, 0);
    stagingBuffer.sendIndex = 0;
    sendNextByte();
}

// Pop next telem_block from outgoing buffer, null if no telemetry
struct telem_block *popOutboundTelemetryQueue(void) {
    struct telem_block* telem = outBuffer[outbuff_start];
    outbuff_start += 1;
    outBuffer[outbuff_start - 1] = 0;
    outbuff_start %= OUTBOUND_QUEUE_SIZE;
    return telem;
}

//TODO: Implement method to split telemetry data between multiple packets

// generate an api string into the given char array from the telem block
// Does not check array length!
unsigned int generateApiHeader(unsigned char *apiString, char dataFrame) {
    unsigned int apiIndex = 0;
    unsigned int length = API_HEADER_LENGTH - API_HEADER_PREFIX + sizeof(struct telem_block);
    //unsigned int telemLength = sizeof(struct telem_block);

    // API Mode header
    apiString[apiIndex++] = 0x7E;
    // Packet length (can't be more than 100bytes anyway, so length MSB = 0)
    apiString[apiIndex++] = (0 & 0xFF);         // MSB  = 0
    apiString[apiIndex++] = (length & 0x00FF);  // LSB <= 100

    // API Identifier
    apiString[apiIndex++] = TRANSMIT_16BIT;
    // Data frame
    apiString[apiIndex++] = dataFrame;
    // Receiver address
    apiString[apiIndex++] = RECEIVER_ADDRESS_MSB;
    apiString[apiIndex++] = RECEIVER_ADDRESS_LSB;
    // Option byte
    apiString[apiIndex++] = OPTION_BYTE;

    return apiIndex;
}

// Send a telemetry block immediately (blocking)
int sendTelemetryBlock(struct telem_block *telem) {

    unsigned char apiHeader[API_HEADER_LENGTH];
    unsigned int headerLength = generateApiHeader(apiHeader, 0);
    // Treat the telemetry block as a character array.
    unsigned char *telemAsArray = (unsigned char*)telem;

    unsigned int lengthCheck = 0;
    unsigned char checksum = 0;
    unsigned int i;
    // Send the header
    for (i = 0; i < headerLength; i++ ) {
        // Culmulative checksum
        if (i >= 3) {
            checksum += apiHeader[i] & 0xFF;
        }
        // Wait for data link to clear from previous send
        while (U2STAbits.TRMT == 0);
        // Load transmit register
        U2TXREG = apiHeader[i];
    }
    lengthCheck += i;
    // Send the telemetry
    for (i = 0; i < sizeof(struct telem_block); i++) {
        // Culmulative checksum
        checksum += telemAsArray[i] & 0xFF;
        // Wait for data link to clear from previous send
        while (U2STAbits.TRMT == 0);
        // Load transmit register
        U2TXREG = telemAsArray[i];
    }
    lengthCheck += i;
    // Send the checksum
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0xFF - checksum;
    lengthCheck += 1;

    // Note: We send the last piece of data through UART and
    // then forget about it. We assume it will get handled eventually
    return 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void) {
    // Short circuit if nothing in the staging area yet
    if ( stagingBuffer.telemetry.asStruct == 0 ) {
        IFS1bits.U2TXIF = 0;
        return;
    }
    sendNextByte();
}
