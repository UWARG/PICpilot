
// The net.* files must be in aircraft code and the base station.

#define OUTBOUND_QUEUE_SIZE 50
#define INBOUND_QUEUE_SIZE 100
#define DEFAULT_SEND_LIMIT 50

struct telem_block {
    long millis;        // Timestamp relative to start echelon
    long long lat, lon; // Latitude and longitude from gps
    float pitch, yaw, roll;
    float pitch_gain, yaw_gain, roll_gain;
};


// Create a zero initialized telem block returns null if fails
struct telem_block *createDataBlock(void);
// Destroy a dataBlock
void destroyDataBlock(struct telem_block *data);

// Add a telem_block to the outbound data queue
// Returns the position in the queue or -1 if no room in queue
int addToOutboundDataQueue(struct telem_block *data);
// Get the number of items waiting to be sent
int getOutboundQueueLength(void);
// Clear all telem blocks waiting to be sent
int clearOutboundDataQueue(void);
// Send a block of data returns the number blocks sent
int sendData(struct telem_block *data);

// Get the maximum number of telem blocks to send before returning
int getMaxSend(void);
// Set the maximum number of telem blocks to send before returning
void setMaxSend(int max);

// Pop next telem_block from incoming buffer, null if no data
struct telem_block *popDataBlock(void);
