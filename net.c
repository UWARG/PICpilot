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
    UART1_SendString("Repeat");
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
        debugTelemetry->editing_gain = (char)1;
    }

    return debugTelemetry;
}

// Destroy a telemetryBlock

void destroyTelemetryBlock(struct telem_block *telem) {
    free(telem);
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

// generate an api string into the given char array from the telem block
// Does not check array length!

unsigned int generateApiString(unsigned char *apiString,
        struct telem_block *telem) {
    // TODO: No need to re-init this every time.
    int apiIndex = 0;
    char dataFrame = 0; //0 for now, we don't care about ACKs
    int length = 5+4+sizeof(struct telem_block);
    if (sizeof(struct telem_block) > MAX_PACKET_SIZE){
        //TODO: Implement method to split telemetry data between multiple packets
    }
    apiString[apiIndex] = 0x7E; apiIndex++;
    apiString[apiIndex] = length & 0xFF00; apiIndex++;
    apiString[apiIndex] = length & 0x00FF; apiIndex++;

    /*API STRING STARTS HERE*/

    apiString[apiIndex] = TRANSMIT_16BIT; apiIndex++;
    apiString[apiIndex] = dataFrame; apiIndex++;
    apiString[apiIndex] = RECEIVER_ADDRESS_MSB; apiIndex++;
    apiString[apiIndex] = RECEIVER_ADDRESS_LSB; apiIndex++;
    apiString[apiIndex] = OPTION_BYTE; apiIndex++;
    // Set up api prefix
    apiString[apiIndex] = '$';
    apiIndex++;
    apiString[apiIndex] = 'S';
    apiIndex++;
    apiString[apiIndex] = 'P';
    apiIndex++;
    apiString[apiIndex] = ',';
    apiIndex++;

    // I'M A WIZARD
    char *telemStart = (char*) &apiString[apiIndex];
    memcpy(telemStart, telem, sizeof (struct telem_block));
    apiIndex += sizeof (struct telem_block);

    /*API STRING STARTS ENDS*/
    //Calculate Checksum
    //IF OPTIMIIZATION IS REQUIRED, PRECALCULATE THE HEADER CHECKSUM
    int j = 0;
    unsigned char checksum = 0;
    for (j = PACKET_HEADER_LENGTH; j < apiIndex; j++) {
        checksum += apiString[j] & 0xFF;
    }
    apiString[apiIndex++] = 0xFF - checksum;


    return apiIndex;
    // TODO: Add additional telemetry block parameters to be sent here
}

int createPackets(unsigned int stringLength, unsigned char *apiString, unsigned char **packets, unsigned char *packetLength) {
    int packetIndex = 0;
    unsigned char dataFrame = 0;

    //If the data is to big to fit in one packet, split it
    int nPackets = stringLength / DEFAULT_SEND_LIMIT + 1;

    int i = 0;
    for (i = 0; i < nPackets; i++) {
        if (INCREMENT_DATA_FRAME) {
            dataFrame++;
        }
        int totalLength = stringLength + API_HEADER_LENGTH;
        packets[i][packetIndex++] = 0x7E;
        packets[i][packetIndex++] = totalLength & 0xFF00;
        packets[i][packetIndex++] = totalLength & 0x00FF;
        packets[i][packetIndex++] = TRANSMIT_16BIT;
        packets[i][packetIndex++] = dataFrame;
        packets[i][packetIndex++] = RECEIVER_ADDRESS_MSB;
        packets[i][packetIndex++] = RECEIVER_ADDRESS_LSB;
        packets[i][packetIndex++] = OPTION_BYTE;

        int packetSize = 0;
        for (packetSize = packetIndex; packetSize < stringLength + packetIndex; packetSize++) {
            //Add the data in
            packets[i][packetSize] = apiString[packetSize - packetIndex];
        }
        //Checksum calculation
        //IF OPTIMIIZATION IS REQUIRED, PRECALCULATE THE HEADER CHECKSUM
        int j = 0;
        unsigned char checksum = 0;
        for (j = packetIndex - API_HEADER_LENGTH; j < packetSize; j++) {
            checksum += packets[i][j] & 0xFF;
        }
        packets[i][packetSize++] = 0xFF - checksum;

        packetLength[i] = packetSize;
    }
    return i; //The number of packets in the set
}

// Send a telemetry block

int sendTelemetryBlock(struct telem_block *telem) {
    unsigned char apiString[MAX_PACKET_SIZE + PACKET_HEADER_LENGTH + API_HEADER_LENGTH];
    unsigned char **packets;
    unsigned char packetLength[20];

    //Allocate array space 20xDEFAULT_SEND_LIMIT matrix
    packets = malloc(20 * sizeof *packets);
    int i = 0;
    for (i = 0; i < 20; i++) {
        packets[i] = malloc((DEFAULT_SEND_LIMIT + API_HEADER_LENGTH + 3) * sizeof *packets[i]);
    }

    //Generate the shit
    unsigned int apiLength = generateApiString(apiString, telem);
    unsigned int numPackets = createPackets(apiLength, apiString, packets, packetLength);
    for (i = 0; i < numPackets; i++) {
        // Wait for data link to clear from previous send
        int j = 0;
        for (j = 0; j < packetLength[i]; j++) {
            while (U2STAbits.TRMT == 0);
            U2TXREG = packets[i][j];
        }

    }
    destroyTelemetryBlock(telem);

    for (i = 0; i < 20; i++) {
        free(packets[i]);
    }
    free(packets);
    // Note: We send the last piece of data through UART and
    // then forget about it. We assume it will get handled eventually
     UART1_SendString("Test 3");
    return 0;
}
