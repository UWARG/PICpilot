/**
 * net_outbound.c
 */

#include "net.h"

#include <stdlib.h>
#include "main.h"

struct telem_block *outBuffer [OUTBOUND_QUEUE_SIZE];
int outbuff_start = 0;
int outbuff_end = 0;

struct telem_block *debugTelemetry;

struct telem_buffer stagingBuffer;

int packetCounter = 0;

const unsigned int PACKET_LENGTH = API_HEADER_LENGTH + sizeof(struct telem_block) + 1;

// Create a telem block returns null if fails
struct telem_block *createTelemetryBlock(p_priority packet) {
    struct telem_block *telem = malloc(sizeof (struct telem_block));
    *(int *)(&(telem->type)) = packet;
    packetCounter++;
    return telem;
}

int packetCount(){
    return packetCounter;
}

// Create a telemetry block to use for debugging, only creates one instance
struct telem_block *getDebugTelemetryBlock(p_priority packet) {
    // If the telemetry block does not exist, create it filled with ones
    // of the respective types
    if (debugTelemetry == 0) {
        switch(packet){
        case PRIORITY0:
            debugTelemetry->data.p1_block.lat = 1;
            debugTelemetry->data.p1_block.lon = 1;
            debugTelemetry->data.p1_block.sysTime = 1;
            debugTelemetry->data.p1_block.roll = 1;
            debugTelemetry->data.p1_block.pitch = 1;
            debugTelemetry->data.p1_block.yaw = 1;
            debugTelemetry->data.p1_block.rollRate = 1;
            debugTelemetry->data.p1_block.pitchRate = 1;
            debugTelemetry->data.p1_block.yawRate = 1;
            debugTelemetry->data.p1_block.airspeed = 1;
            debugTelemetry->data.p1_block.alt = 1;
            debugTelemetry->data.p1_block.UTC = 1;
            debugTelemetry->data.p1_block.gSpeed = 1;
            debugTelemetry->data.p1_block.heading = 1;
            debugTelemetry->data.p1_block.rollRateSetpoint = 1;
            debugTelemetry->data.p1_block.rollSetpoint = 1;
            debugTelemetry->data.p1_block.pitchRateSetpoint = 1;
            debugTelemetry->data.p1_block.pitchSetpoint = 1;
            debugTelemetry->data.p1_block.throttleSetpoint = 1;
            break;
        case PRIORITY1:
            debugTelemetry->data.p2_block.rollKD = 1;
            debugTelemetry->data.p2_block.rollKP = 1;
            debugTelemetry->data.p2_block.pitchKD = 1;
            debugTelemetry->data.p2_block.pitchKP = 1;
            debugTelemetry->data.p2_block.yawKD = 1;
            debugTelemetry->data.p2_block.yawKP = 1;
            debugTelemetry->data.p2_block.lastCommandsSent[0] = 1;
            debugTelemetry->data.p2_block.lastCommandsSent[1] = 1;
            debugTelemetry->data.p2_block.lastCommandsSent[2] = 1;
            debugTelemetry->data.p2_block.lastCommandsSent[3] = 1;
            debugTelemetry->data.p2_block.batteryLevel1 = 1;
            //debugTelemetry->data.p2_block.batteryLevel2 = 1;
            debugTelemetry->data.p2_block.ch1In = 1;
            debugTelemetry->data.p2_block.ch2In = 1;
            debugTelemetry->data.p2_block.ch3In = 1;
            debugTelemetry->data.p2_block.ch4In = 1;
            debugTelemetry->data.p2_block.ch5In = 1;
            debugTelemetry->data.p2_block.ch6In = 1;
            debugTelemetry->data.p2_block.ch7In = 1;
            debugTelemetry->data.p2_block.ch8In = 1;
            debugTelemetry->data.p2_block.ch1Out = 1;
            debugTelemetry->data.p2_block.ch2Out = 1;
            debugTelemetry->data.p2_block.ch3Out = 1;
            debugTelemetry->data.p2_block.ch4Out = 1;
            debugTelemetry->data.p2_block.ch5Out = 1;
            debugTelemetry->data.p2_block.ch6Out = 1;
            debugTelemetry->data.p2_block.ch7Out = 1;
            debugTelemetry->data.p2_block.ch8Out = 1;
            debugTelemetry->data.p2_block.yawRateSetpoint = 1;
            debugTelemetry->data.p2_block.headingSetpoint = 1;
            debugTelemetry->data.p2_block.altitudeSetpoint = 1;
            debugTelemetry->data.p2_block.flapSetpoint = 1;
            debugTelemetry->data.p2_block.cameraStatus = 1;
            debugTelemetry->data.p2_block.wirelessConnection = 1;
            debugTelemetry->data.p2_block.autopilotActive = 1;
            debugTelemetry->data.p2_block.gpsStatus = 1;
            //debugTelemetry->data.p2_block.pathChecksum = 1;
            debugTelemetry->data.p2_block.numWaypoints = 1;
            debugTelemetry->data.p2_block.waypointIndex = 1;
            //debugTelemetry->data.p2_block.following = 1;
            break;
        case PRIORITY2:
            debugTelemetry->data.p3_block.rollKI = 1;
            debugTelemetry->data.p3_block.pitchKI = 1;
            debugTelemetry->data.p3_block.yawKI = 1;
            debugTelemetry->data.p3_block.headingKD = 1;
            debugTelemetry->data.p3_block.headingKP = 1;
            debugTelemetry->data.p3_block.headingKI = 1;
            debugTelemetry->data.p3_block.altitudeKD = 1;
            debugTelemetry->data.p3_block.altitudeKP = 1;
            debugTelemetry->data.p3_block.altitudeKI = 1;
            debugTelemetry->data.p3_block.throttleKD = 1;
            debugTelemetry->data.p3_block.throttleKP = 1;
            debugTelemetry->data.p3_block.throttleKI = 1;
            debugTelemetry->data.p3_block.flapKD = 1;
            debugTelemetry->data.p3_block.flapKP = 1;
            debugTelemetry->data.p3_block.flapKI = 1;
            debugTelemetry->data.p3_block.startupErrorCodes = 1;
            debugTelemetry->data.p3_block.startupSettings = 1;
            break;

        default:
            break;
        }
    }
    return debugTelemetry;
}

// Destroy a telemetryBlock
void destroyTelemetryBlock(struct telem_block *telem) {
    free(telem);
    telem = 0;
    packetCounter--;
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
void outboundBufferMaintenance(void) {
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
    stagingBuffer.checksum = 0;
    // Send index should be reset last for reasons
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
    // Packet length
    apiString[apiIndex++] = 0; // MSB (Can only go up to 100)
    apiString[apiIndex++] = (length & 0x00FF);      // LSB

    //Frame Type
    apiString[apiIndex++] = TX_PACKET;
    // Data frame
    apiString[apiIndex++] = dataFrame;
    // Receiver address
    apiString[apiIndex++] = (RECEIVER_ADDRESS & 0xFF00000000000000) >> 56;
    apiString[apiIndex++] = (RECEIVER_ADDRESS & 0x00FF000000000000) >> 48;
    apiString[apiIndex++] = (RECEIVER_ADDRESS & 0x0000FF0000000000) >> 40;
    apiString[apiIndex++] = (RECEIVER_ADDRESS & 0x000000FF00000000) >> 32;
    apiString[apiIndex++] = (RECEIVER_ADDRESS & 0x00000000FF000000) >> 24;
    apiString[apiIndex++] = (RECEIVER_ADDRESS & 0x0000000000FF0000) >> 16;
    apiString[apiIndex++] = (RECEIVER_ADDRESS & 0x000000000000FF00) >> 8;
    apiString[apiIndex++] = (RECEIVER_ADDRESS & 0x00000000000000FF);

    //Reserved Data
    apiString[apiIndex++] = 0xFF;
    apiString[apiIndex++] = 0xFE;
//    *((long long *)(&apiString[apiIndex])) = RECEIVER_ADDRESS;
//    apiIndex += 8; //Receiver Address is 64 bit
    //Node Path distance
    apiString[apiIndex++] = BROADCAST_RADIUS;
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
        if (i >= API_HEADER_PREFIX) {
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
