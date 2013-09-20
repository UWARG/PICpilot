#include "net.h"
#include <stdlib.h>

struct telem_block *outBuffer
    [OUTBOUND_QUEUE_SIZE];
int outbuff_start = 0;
int outbuff_end = 0;

struct telem_block *inBuffer [INBOUND_QUEUE_SIZE];
int inbuff_start = 0;
int inbuff_end = 0;

int maxSend = DEFAULT_SEND_LIMIT;

// Send a telemetry block
int sendBlock(struct telem_block *telem) {
    return 0;
}

// Create a zero initialized telem block returns null if fails
struct telem_block *createtelemetryBlock(void) {
    struct telem_block *telemetry = malloc(sizeof(struct telem_block));
    return telemetry;
}

// Destroy a telemetryBlock
void destroytelemetryBlock(struct telem_block *telemetry) {
    free( telemetry);
}

// Add a telem_block to the outbound telemetry queue
// Returns the position in the queue or -1 if no room in queue
int addToOutboundtelemetryQueue(struct telem_block *telemetry) {
    outBuffer[outbuff_end] = telemetry;
    outbuff_end++;
    outbuff_end = outbuff_end % OUTBOUND_QUEUE_SIZE;
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
int clearOutboundtelemetryQueue(void) {
    int i = 0;
    for (i = 0; i < OUTBOUND_QUEUE_SIZE; i++) {
        outBuffer[i] = 0;
    }
}

// Send a block of telemetry returns the number of blocks sent
int sendtelemetry(struct telem_block *telemetry) {
    int blocks_sent;
    int failed_blocks = 0;
    int sent = 0;       // 1 - yes, 0 - no
    for (blocks_sent = 0; blocks_sent < maxSend; blocks_sent++) {
        sent = sendBlock(outBuffer[outbuff_start]);
        if (! sent) {
            blocks_sent--;
            failed_blocks++;
        }
        if (failed_blocks > 0) {
            return 1;
        }
    }
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
