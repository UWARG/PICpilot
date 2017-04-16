/**
 * @file Datalink.h
 * @author Chris Hajduk, Serj Babayan
 * @date September 2015
 * @brief List of defines and configuration for datalink
 * @copyright Waterloo Aerial Robotics Group 2016-2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef DATALINK_H
#define	DATALINK_H

#include <stdint.h>
#include <stdbool.h>

/** Time in miliseconds for how often a P0(high priority) packet gets sent down. Default=300 **/
#define P0_SEND_FREQUENCY 250 

/** Time in miliseconds for how often a P1(medium priority) packet gets sent down. Default=1000 **/
#define P1_SEND_FREQUENCY 1000

/** Time in miliseconds for how often a P2(low priority) packet gets sent down. Default=20000 **/
#define P2_SEND_FREQUENCY 5000

/** Time in miliseconds for how often to check for new messages from the uplink. Default=100 **/
#define UPLINK_CHECK_FREQUENCY 500

// TODO: Put these headers in the new kill mode implementation
#define HEARTBEAT_TIMEOUT 10000 //In Milliseconds
    
#define GPS_TIMEOUT 30000 //In Milliseconds

#define HEARTBEAT_KILL_TIMEOUT 20000 //In Milliseconds

#define UHF_KILL_TIMEOUT 10000

typedef enum _p_priority {
    PRIORITY0 = 0,
    PRIORITY1 = 1,
    PRIORITY2 = 2,
} p_priority;


/* For reference: 
 In MPLAB XC 16 compiler:
 char           : 1 byte
 int            : 2 bytes
 long int       : 4 bytes
 float          : 4 bytes
 long double    : 8 bytes
 */

// 72 bytes
struct priority1_block { //High Frequency - Multiple times per second
    long double lat, lon; // Latitude and longitude from gps    // 2x8 Byte
    long int sysTime; // 4 bytes
    float UTC; //4 Byte
    float pitch, roll, yaw;
    float pitchRate, rollRate, yawRate;
    float airspeed;
    float alt; //4 Byte
    float gSpeed;
    int heading; //2 Byte
    int rollRateSetpoint, rollSetpoint;
    int pitchRateSetpoint, pitchSetpoint;
    int throttleSetpoint;
};

// 88 bytes
struct priority2_block { //Medium Frequency - Once every second
    float rollKD, rollKP;
    float pitchKD, pitchKP;
    float yawKD, yawKP;
    float pathChecksum; // 4 bytes
    int lastCommandsSent[4]; //4*2 bytes
    int batteryLevel1, batteryLevel2; // 2*2 bytes
    int ch1In,ch2In,ch3In,ch4In,ch5In,ch6In,ch7In,ch8In;
    int ch1Out,ch2Out,ch3Out,ch4Out,ch5Out,ch6Out,ch7Out,ch8Out;
    int cameraStatus;
    int yawRateSetpoint, headingSetpoint, altitudeSetpoint, flapSetpoint;
    char wirelessConnection; //1 byte
    char autopilotActive; //1 byte  
    char gpsStatus; //1 Byte
    char numWaypoints; //1 bytes
    char waypointIndex; //1 byte
    char pathFollowing; // 1 byte
};

// 74 bytes
struct priority3_block { //Low Frequency - On update...
    float rollKI; //4 Bytes
    float pitchKI;
    float yawKI;
    float headingKD, headingKP, headingKI;
    float altitudeKD, altitudeKP, altitudeKI;
    float throttleKD, throttleKP, throttleKI;
    float flapKD, flapKP, flapKI;
    float pathGain, orbitGain;
    int autonomousLevel;
    unsigned int startupErrorCodes; //2 bytes
    int startupSettings;
    uint16_t dl_transmission_errors, ul_receive_errors;
    uint8_t ul_rssi, uhf_rssi, uhf_link_quality;
};

typedef union {
    struct priority1_block p1_block;
    struct priority2_block p2_block;
    struct priority3_block p3_block;
} PacketPayload;

typedef struct TelemetryBlock {
    uint8_t type;
    PacketPayload data;
} TelemetryBlock;

typedef struct DatalinkCommand {
    uint8_t cmd;
    uint8_t data_length;
    uint8_t* data;
    struct DatalinkCommand* next;
} DatalinkCommand;

/**
 * Initializes the data link connection. Initializes the specified Radio as 
 * well as sets up the internal command buffer
 * @return Whether successfully initialized
 */
void initDatalink(void);

/**
 * Reads the received data from the radio. If the received data was a TX sent
 * from the ground station, will parse the command and add to the internal command
 * queue. This function should be called whenever possible, as it is non-blocking,
 * and will add at most a single command
 */
void parseDatalinkBuffer(void);

/**
 * Pop a command from the internal command queue
 * @return The command, or NULL if there are commands available
 */
DatalinkCommand* popDatalinkCommand(void);

/**
 * Destroys a command appropriately. Make sure to use this method rather than
 * calling free() on a Command yourself
 * @param to_destroy
 */
void freeDatalinkCommand(DatalinkCommand* to_destroy);

/**
 * Create a telemetry block
 * @param packet Priority of the packet
 * @return NULL if malloc failed
 */
TelemetryBlock* createTelemetryBlock(p_priority packet);

/**
 * Queues a telemetry block to be sent down the data link
 * @param data
 * @return True if successfully queued, false otherwise
 */
bool queueTelemetryBlock(TelemetryBlock* data);

#endif
