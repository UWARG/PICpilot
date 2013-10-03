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
    struct telem_block *telem = malloc(sizeof(struct telem_block));
    return telem;
}

// Create a telemetry block to use for debugging, only creates one instance
struct telem_block *getDebugTelemetryBlock(void) {
    // If the telemetry block does not exist, create it with
    // alternating ones and zeros (0xAA)
    if (debugTelemetry == 0) {
        debugTelemetry = createTelemetryBlock();
        debugTelemetry->millis = (long long) 0xAAAAAAAAAAAAAAAA;
        debugTelemetry->lat = (long double) 0xAAAAAAAAAAAAAAAA;
        debugTelemetry->lon = (long double) 0xAAAAAAAAAAAAAAAA;
        debugTelemetry->pitch = (float) 0xAAAAAAAA;
        debugTelemetry->roll = (float) 0xAAAAAAAA;;
        debugTelemetry->yaw = (float) 0xAAAAAAAA;
        debugTelemetry->pitchRate = (float) 0xAAAAAAAA;
        debugTelemetry->rollRate = (float) 0xAAAAAAAA;
        debugTelemetry->yawRate = (float) 0xAAAAAAAA;
        debugTelemetry->pitchSetpoint = (int) 0xAAAA;
        debugTelemetry->rollSetpoint = (int) 0xAAAA;
        debugTelemetry->yawSetpoint = (int) 0xAAAA;
        debugTelemetry->pitch_gain = (float) 0xAAAAAAAA;
        debugTelemetry->roll_gain = (float) 0xAAAAAAAA;
        debugTelemetry->yaw_gain = (float) 0xAAAAAAAA;
        debugTelemetry->editing_gain = (char) 0xAA;
    }
    return debugTelemetry;
}

// Destroy a telemetryBlock
void destroyTelemetryBlock(struct telem_block *telem) {
    free( telem);
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
    int sent = 0;       // 1 - yes, 0 - no
    for (blocks_sent = 0; blocks_sent < maxSend; blocks_sent++) {
        sent = sendTelemetryBlock(outBuffer[outbuff_start]);
        if (! sent) {
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

// generate an api string into the given char array from the telem block
// Does not check array length!
int generateApiString(unsigned char *apiString,
        struct telem_block *telem) {
    // TODO: No need to re-init this every time.
    int apiIndex = 0;
    apiString[apiIndex] = 0x7E; apiIndex++;
    apiString[apiIndex] = 0x00; apiIndex++;
    apiString[apiIndex] = (char) (5+4+sizeof(struct telem_block)); apiIndex++;
    apiString[apiIndex] = 0x01; apiIndex++;
    apiString[apiIndex] = 0x00; apiIndex++;
    apiString[apiIndex] = 0xCC; apiIndex++;
    apiString[apiIndex] = 0xCC; apiIndex++;
    apiString[apiIndex] = 0x01; apiIndex++;
    // Set up api prefix
    apiString[apiIndex] = '$';  apiIndex++;
    apiString[apiIndex] = 'S';  apiIndex++;
    apiString[apiIndex] = 'P';  apiIndex++;
    apiString[apiIndex] = ',';  apiIndex++;

    // I'M A WIZARD
    char *telemStart = (char*) &apiString[13];
    memcpy(telemStart, telem, sizeof(struct telem_block));
    apiIndex += sizeof(struct telem_block);
    return apiIndex;
    // TODO: Add additional telemetry block parameters to be sent here
}

// Send a telemetry block
int sendTelemetryBlock(struct telem_block *telem) {
    unsigned char apiString[100];

    int apiLength = generateApiString(apiString, telem);
    int i;
    for (i = 0; i < apiLength; i++) {
        // Wait for data link to clear from previous send
        while(U2STAbits.TRMT == 0);
        U2TXREG = apiString[i];

    }
    // Note: We send the last piece of data through UART and
    // then forget about it. We assume it will get handled eventually
    return 0;
}
