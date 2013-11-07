#include "main.h"
#include "net.h"
#include <stdlib.h>
#include <string.h>
#include "p33FJ256GP710.h"
#include "UART2.h"

struct telem_block *outBuffer
[OUTBOUND_QUEUE_SIZE];
int outbuff_start = 0;
int outbuff_end = 0;

struct telem_block *inBuffer [INBOUND_QUEUE_SIZE];
int inbuff_start = 0;
int inbuff_end = 0;

int maxSend = DEFAULT_SEND_LIMIT;

struct telem_block *debugTelemetry;

// Initialize the data link

int initDataLink(void) {
    InitUART2();
    return 0;
}

// Create a zero initialized telem block returns null if fails

struct telem_block *createTelemetryBlock(void) {
    struct telem_block *telem = malloc(sizeof (struct telem_block));
    return telem;
}

// Create a telemetry block to use for debugging, only creates one instance

struct telem_block *getDebugTelemetryBlock(void) {
    // If the telemetry block does not exist, create it with
    // alternating ones and zeros (0xAA)
    if (debugTelemetry == 0) {
        debugTelemetry = createTelemetryBlock();
        unsigned char *debugTelemetryArray = debugTelemetry;
        int i;
        for ( i = 0; i < sizeof(struct telem_block); i++) {
            if ( i % 8 == 0)
                debugTelemetryArray[i] = 0xAA;
            else if ( i % 8 == 1)
                debugTelemetryArray[i] = 0xBB;
            else if ( i % 8 == 2)
                debugTelemetryArray[i] = 0xCC;
            else if ( i % 8 == 3)
                debugTelemetryArray[i] = 0xDD;
            else if ( i % 8 == 4)
                debugTelemetryArray[i] = 0xEE;
        }
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

int addToOutboundTelemetryQueue(struct telem_block *telem) {
    int currentQueueLength = getOutboundQueueLength();
    if (currentQueueLength - OUTBOUND_QUEUE_SIZE <= 0) {
        return -1;
    }
    outBuffer[outbuff_end] = telem;
    outbuff_end++;
    outbuff_end = outbuff_end % OUTBOUND_QUEUE_SIZE;
    return currentQueueLength;
}

// Get the number of items waiting to be sent

int getOutboundQueueLength(void) {
    if (outbuff_end < outbuff_start) {
        return outbuff_end - outbuff_start + OUTBOUND_QUEUE_SIZE;
    } else {
        return outbuff_end - outbuff_start;
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

// Send a block of telemetry returns the number of blocks sent

int sendTelemetry(struct telem_block *telemetry) {
    int blocks_sent = 0;
    int failed_blocks = 0;
    int sent = 0; // 1 - yes, 0 - no
    for (blocks_sent = 0; blocks_sent < maxSend; blocks_sent++) {
        sent = sendTelemetryBlock(outBuffer[outbuff_start]);
        if (!sent) {
            blocks_sent--;
            failed_blocks++;
        }
        if (failed_blocks > 0) {
            return -1;
        }
    }
    return blocks_sent;
}

// Get the maximum number of telem blocks to send before returning

int getMaxSend(void) {
    return maxSend;
}

// Set the maximum number of telem blocks to send before returning

void setMaxSend(int max) {
    maxSend = max;
}

// Pop next telem_block from incoming buffer, null if no telemetry

struct telem_block *popTelemetryBlock(void) {
    return (void *) 0;
}

//TODO: Implement method to split telemetry data between multiple packets

// generate an api string into the given char array from the telem block
// Does not check array length!
unsigned int generateApiHeader(unsigned char *apiString, char dataFrame) {
    unsigned int apiIndex = 0;
    unsigned int length = API_HEADER_LENGTH + sizeof(struct telem_block);
    unsigned int telemLength = sizeof(struct telem_block);

    // API Mode header
    apiString[apiIndex++] = 0x7E;
    // Packet length
    apiString[apiIndex] = (length & 0xFF00); apiIndex++;
    apiString[apiIndex] = (length & 0x00FF); apiIndex++; // XXX: Big endian/little endian?

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

// Send a telemetry block
int sendTelemetryBlock(struct telem_block *telem) {

    unsigned char apiHeader[API_HEADER_LENGTH];
    unsigned int headerLength = generateApiHeader(apiHeader, 0);
    unsigned int packetLength = API_HEADER_LENGTH + sizeof(struct telem_block) + 1 + PACKET_HEADER_LENGTH;
    // Treat the telemetry block as a character array.
    unsigned char *telemAsArray = telem;
    unsigned char debugTelemArray[sizeof(struct telem_block)];

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
        debugTelemArray[i] = telemAsArray[i] & 0xFF;
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
    if ( packetLength != lengthCheck && DEBUG) {
        UART1_SendString("Length Mismatch");
    }
    return 0;
}
