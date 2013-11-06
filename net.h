
// The net.* files must be in aircraft code and the base station.

#define OUTBOUND_QUEUE_SIZE 50
#define INBOUND_QUEUE_SIZE 100
#define DEFAULT_SEND_LIMIT 50
#define MAX_PACKET_SIZE 100

#define EDIT_NONE 0
#define EDIT_PITCH_GAIN 1
#define EDIT_ROLL_GAIN 2
#define EDIT_YAW_GAIN 3

#define CHECKSUM_START_OFFSET 3
#define LENGTH_START_OFFSET 3
#define LENGTH_POSITION 2
#define API_HEADER_LENGTH 5
#define PACKET_HEADER_LENGTH 3

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
    int pitchSetpoint, rollSetpoint, yawSetpoint;   // 2Byte
    char editing_gain;                              // 1Byte
    // TODO: Add additional telemetry to be sent here
};

// Initialize the data link
int initDataLink(void);

// Create a telemetry block to use for debugging, only creates one instance
struct telem_block *getDebugTelemetryBlock(void);

// Create a zero initialized telem block returns null if fails
struct telem_block *createTelemetryBlock(void);
// Destroy a dataBlock
void destroyTelemetryBlock(struct telem_block *data);

// Add a telem_block to the outbound data queue
// Returns the position in the queue or -1 if no room in queue
int addToOutboundTelemetryQueue(struct telem_block *data);
// Get the number of items waiting to be sent
int getOutboundQueueLength(void);
// Clear all telem blocks waiting to be sent
int clearOutboundTelemetryQueue(void);
// Send a block of data returns the number blocks sent
int sendTelemetry(struct telem_block *telem);

// Get the maximum number of telem blocks to send before returning
int getMaxSend(void);
// Set the maximum number of telem blocks to send before returning
void setMaxSend(int max);

// Pop next telem_block from incoming buffer, null if no data
struct telem_block *popTelemetryBlock(void);

// Send a block of telemetry probably should call this directly
int sendTelemetryBlock(struct telem_block *telem);
