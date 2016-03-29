/**
 * net.h
 */

#ifndef NET_H
#define	NET_H

#ifdef	__cplusplus
extern "C" {
#endif

#define DATALINK_SEND_FREQUENCY 100 //Time in milliseconds
#define UPLINK_CHECK_FREQUENCY 100 //Time in milliseconds

#define BLOCKING_MODE 0

#define OUTBOUND_QUEUE_SIZE 20
#define INBOUND_QUEUE_SIZE 100
#define MAX_PACKET_SIZE 20

#define EDIT_NONE 0
#define EDIT_PITCH_GAIN 1
#define EDIT_ROLL_GAIN 2
#define EDIT_YAW_GAIN 3

#define API_HEADER_LENGTH 17
#define API_HEADER_PREFIX 3

#define RECEIVER_ADDRESS 0x0013A20040B47E6B//37745 //B47E6B

#define INCREMENT_DATA_FRAME 0x00
#define OPTION_BYTE 0x01 //Disables ACK

//FRAME TYPE
#define TX_PACKET 0x10
#define BROADCAST_RADIUS 1 //0 is infinite number of hops to reach target
#define RAW_PACKET_BUFFER_SIZE 16   // Number of buffer regions to allow for incoming commands

#define HEARTBEAT_TIMEOUT 10000 //In Milliseconds

#define GPS_TIMEOUT 30000 //In Milliseconds

#define HEARTBEAT_KILL_TIMEOUT 120000 //In Milliseconds

#define UHF_KILL_TIMEOUT_FAST 10000

typedef enum _p_priority {
    PRIORITY1 = 1,
    PRIORITY2 = 2,
    PRIORITY3 = 3
} p_priority;

struct priority1_block { //High Frequency - Multiple times per second
    long double lat, lon; // Latitude and longitude from gps    // 8Byte
    long int sysTime; // 8 bytes
    float UTC; //4 Byte
    float pitch, roll, yaw;
    float pitchRate, rollRate, yawRate;
    float airspeed;
    float alt; //4 Byte
    float gSpeed;
    int heading; //2 Byte
};

struct priority2_block { //Medium Frequency - Once every second
    int lastCommandSent; //4 bytes
    int batteryLevel1, batteryLevel2; // 4 bytes
    unsigned int startupErrorCodes; //2 bytes
    int ch1In,ch2In,ch3In,ch4In,ch5In,ch6In,ch7In,ch8In;
    int ch1Out,ch2Out,ch3Out,ch4Out,ch5Out,ch6Out,ch7Out,ch8Out;
    int rollRateSetpoint, rollSetpoint, pitchRateSetpoint, pitchSetpoint, throttleSetpoint, yawRateSetpoint, headingSetpoint, altitudeSetpoint, flapSetpoint;
    char wirelessConnection; //1 byte
    char autopilotActive; //1 byte  
    char gpsStatus; //1 Byte
    char pathChecksum; //1 byte
    char numWaypoints; //1 bytes
    char waypointIndex; //1 byte
    char pathFollowing; // 1 byte

};

struct priority3_block { //Low Frequency - On update...
    float rollKD, rollKP, rollKI; //4 Bytes
    float pitchKD, pitchKP, pitchKI;
    float yawKD, yawKP, yawKI;
    float headingKD, headingKP, headingKI;
    float altitudeKD, altitudeKP, altitudeKI;
    float throttleKD, throttleKP, throttleKI;
    float flapKD, flapKP, flapKI;
    float pathGain, orbitGain;
    int cameraStatus;
};

typedef union {
    struct priority1_block p1_block;
    struct priority2_block p2_block;
    struct priority3_block p3_block;
} packetPayload;

struct telem_block {
    const int type;
    packetPayload data;
};

//struct telem_block {
//    long double lat, lon; // Latitude and longitude from gps    // 8Byte
//    float millis;        // Timestamp UTC  // 4Byte
//    float pitch, roll, yaw;                         // 4Byte
//    float pitchRate, rollRate, yawRate;             // 4Byte
//    float kd_gain, kp_gain, ki_gain;          // 4Byte
//    float groundSpeed;
////    float airspeed;
//    float altitude;
//    int heading;
//    int pitchSetpoint, rollSetpoint, headingSetpoint, throttleSetpoint, flapSetpoint; //Angle
//    int altitudeSetpoint;
//    int cPitchSetpoint, cRollSetpoint, cYawSetpoint;  //Controller input // 2Byte
//    int lastCommandSent;
//    int errorCodes;
//    int cameraStatus;
//    int airspeed;
//    char waypointIndex;
//    char editing_gain, gpsStatus, batteryLevel, waypointCount;                              // 1Byte
//    // TODO: Add additional telemetry to be sent here
//};

struct telem_buffer {
    unsigned int sendIndex;             // index into telemetry to send
    unsigned char header[API_HEADER_LENGTH];    // The header for the telem
    union {
        struct telem_block *asStruct;   // The telemetry block being sent
        unsigned char *asArray;         // The telemetry intepreted as an array
    } telemetry;
    unsigned char checksum;             // The checksum so far
};

struct command {
    unsigned char cmd;
    unsigned char data_length;
    unsigned char data[101];
};

// Initialize the data link
int initDataLink(void);

// Create a telemetry block to use for debugging, only creates one instance
struct telem_block *getDebugTelemetryBlock(p_priority packet);

// Create a telem block returns null if fails
struct telem_block *createTelemetryBlock(p_priority packet);
// Destroy a dataBlock
void destroyTelemetryBlock(struct telem_block *data);

// Add a telem_block to the outbound data queue
// Returns the position in the queue or -1 if no room in queue
int pushOutboundTelemetryQueue(struct telem_block *data);
// Get the number of items waiting to be sent
int getOutboundQueueLength(void);
// Clear all telem blocks waiting to be sent
int clearOutboundTelemetryQueue(void);

// Pop next telem_block from incoming buffer, null if no data
struct telem_block *popOutboundTelemetryQueue(void);

// generate the header for the a telemetry block
unsigned int generateApiHeader(unsigned char *apiString, char dataFrame);

// Stage the given telemetry block
void stageTelemetryBlock(struct telem_block *telem);
// Do outbound buffer maintenance
void outboundBufferMaintenance(void);

// Send a block of telemetry immediately, probably shouldn't call this directly
int sendTelemetryBlock(struct telem_block *telem);

/****************************************************************************
 * Inbound relevant functions
 */

// Clean up the command
void destroyCommand( struct command* cmd );

// Get the next command
struct command* popCommand();

// Queue up another command
int pushCommand(struct command* cmd);

// Create a new command struct
struct command* createCommand( char* rawPacket );

// Check and make sure the char array is a valid packet
int checkPacket( char* rawPacket);

// Handle the inbound buffer
void inboundBufferMaintenance(void);

#ifdef	__cplusplus
}
#endif

#endif	/* NET_H */
