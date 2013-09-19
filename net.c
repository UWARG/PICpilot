#include "net.h"
#include <stdlib.h>

struct data_block *outBuffer
    [OUTBOUND_QUEUE_SIZE];
int outbuff_start = 0;
int outbuff_end = 0;

struct data_block *inBuffer [INBOUND_QUEUE_SIZE];
int inbuff_start = 0;
int inbuff_end = 0;

int maxSend = DEFAULT_SEND_LIMIT;

// Send a data block
int sendBlock(struct data_block *data) {
    return 0;
}

// Create a zero initialized data block returns null if fails
struct data_block *createDataBlock(void) {
    struct data_block *data = malloc(sizeof(struct data_block));
    return data;
}

// Destroy a dataBlock
void destroyDataBlock(struct data_block *data) {
    free( data);
}

// Add a data_block to the outbound data queue
// Returns the position in the queue or -1 if no room in queue
int addToOutboundDataQueue(struct data_block *data) {
    outBuffer[outbuff_end] = data;
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

// Clear all data blocks waiting to be sent
int clearOutboundDataQueue(void) {
    int i = 0;
    for (i = 0; i < OUTBOUND_QUEUE_SIZE; i++) {
        outBuffer[i] = 0;
    }
}

// Send a block of data returns the number of blocks sent
int sendData(struct data_block *data) {
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

// Get the maximum number of data blocks to send before returning
int getMaxSend(void) {
    return maxSend;
}

// Set the maximum number of data blocks to send before returning
void setMaxSend(int max) {
    maxSend = max;
}

// Pop next data_block from incoming buffer, null if no data
struct data_block *popDataBlock(void) {
}
