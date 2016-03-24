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

#define PACKET_ATTITUDE 1
#define PACKET_STATUS 2
#define PACKET_ERRORS 3
#define PACKET_GAIN 4
#define PACKET_INPUTS 5
#define PACKET_SETPOINTS 6
#define PACKET_OUTPUTS 7
#define PACKET_LOCATION 8
#define PACKET_CAMERA 9

struct attitude_block {
    const int type = PACKET_ATTITUDE;
    float pitch, roll, yaw;
    float pitchRate, rollRate, yawRate;
};

struct status_block {
    const int type = PACKET_STATUS;
    long int sysTime; // 8 bytes
};

struct gain_block {
    const int type = PACKET_GAIN;
    float rollKD, rollKP, rollKI; //4 Bytes
    float pitchKD, pitchKP, pitchKI;
    float yawKD, yawKP, yawKI;


};

struct setpoint_block {
    const int type = PACKET_SETPOINTS;
    int altitudeSetpoint;
    int pitchSetpoint, rollSetpoint, headingSetpoint, throttleSetpoint, flapSetpoint; //Angle
};

struct output_block {
    const int type = PACKET_OUTPUTS;
};

struct location_block {
    const int type = PACKET_LOCATION;
    long double lat, lon; // Latitude and longitude from gps    // 8Byte
    float alt; //4 Byte
    float UTC; //4 Byte

};

struct camera_block {
    const int type = PACKET_CAMERA;
    int cameraStatus;
};

union telem_block {
    struct attitude_block att_block;
    struct status_block stat_block;
//    struct error_block err_block;
//    struct input_block in_block;
    struct gain_block g_block;
    struct setpoint_block  spoint_block;
    struct output_block out_block;
    struct location_block  loc_block;
    struct camera_block cam_block;
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
struct telem_block *getDebugTelemetryBlock(void);

// Create a telem block returns null if fails
struct telem_block *createTelemetryBlock(void);
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
