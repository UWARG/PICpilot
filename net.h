
// The net.* files must be in aircraft code and the base station.

#define OUTBOUND_QUEUE_SIZE 50
#define INBOUND_QUEUE_SIZE 100
#define DEFAULT_SEND_LIMIT 50

struct data_block {
    long millis;
    long long lat, lon;
    double pitch, yaw, roll;
};


// Create a zero initialized data block returns null if fails
struct data_block *createDataBlock(void);
// Destroy a dataBlock
void destroyDataBlock(struct data_block *data);

// Add a data_block to the outbound data queue
// Returns the position in the queue or -1 if no room in queue
int addToOutboundDataQueue(struct data_block *data);
// Get the number of items waiting to be sent
int getOutboundQueueLength(void);
// Clear all data blocks waiting to be sent
int clearOutboundDataQueue(void);
// Send a block of data returns the number blocks sent
int sendData(struct data_block *data);

// Get the maximum number of data blocks to send before returning
int getMaxSend(void);
// Set the maximum number of data blocks to send before returning
void setMaxSend(int max);

// Pop next data_block from incoming buffer, null if no data
struct data_block *popDataBlock(void);
