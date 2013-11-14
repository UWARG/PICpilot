
// The net.* files must be in aircraft code and the base station.

#define BLOCKING_MODE 1

#define OUTBOUND_QUEUE_SIZE 20
#define INBOUND_QUEUE_SIZE 40
#define MAX_PACKET_SIZE 100

#define EDIT_NONE 0
#define EDIT_PITCH_GAIN 1
#define EDIT_ROLL_GAIN 2
#define EDIT_YAW_GAIN 3

#define API_HEADER_LENGTH 8
#define API_HEADER_PREFIX 3

#define RECEIVER_ADDRESS_MSB 0x50
#define RECEIVER_ADDRESS_LSB 0x01

#define INCREMENT_DATA_FRAME 0x00
#define OPTION_BYTE 0x01 //Disables ACK
//API IDENTIFIER TYPES
#define TRANSMIT_64BIT 0x00
#define TRANSMIT_16BIT 0x01


struct telem_block {
    long long millis;        // Timestamp relative to start echelon  // 8Byte
    long double lat, lon; // Latitude and longitude from gps    // 8Byte
    float pitch, roll, yaw;                         // 4Byte
    float pitchRate, rollRate, yawRate;             // 4Byte
    float pitch_gain, roll_gain, yaw_gain;          // 4Byte
    int pitchSetpoint, rollSetpoint, yawSetpoint, throttleSetpoint;   // 2Byte
    char editing_gain;                              // 1Byte
    // TODO: Add additional telemetry to be sent here
};

struct telem_buffer {
    unsigned int sendIndex;             // index into telemetry to send 
    unsigned char header[API_HEADER_LENGTH];    // The header for the telem
    union {
        struct telem_block *asStruct;   // The telemetry block being sent
        unsigned char *asArray;         // The telemetry intepreted as an array
    } telemetry;
    unsigned char checksum;             // The checksum so far
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
// Do buffer maintenance
void bufferMaintenance(void);

// Send a block of telemetry immediately, probably shouldn't call this directly
int sendTelemetryBlock(struct telem_block *telem);
