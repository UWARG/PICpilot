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
#include "Commands.h"

/** Time in ms of how often a packet containing aircraft positional information will be sent*/
#define POSITION_SEND_FREQUENCY 250

/** Time in ms of how often a packet containing autopilot status information will be sent*/
#define STATUS_SEND_FREQUENCY 1000

/** Time in ms of how often a packet containing channel information will be sent*/
#define CHANNELS_SEND_FREQUENCY 2000

/** Time in miliseconds for how often to check for new messages from the uplink. Default=100 **/
#define UPLINK_CHECK_FREQUENCY 500

// TODO: Put these headers in the new kill mode implementation
#define HEARTBEAT_TIMEOUT 10000 //In Milliseconds
    
#define GPS_TIMEOUT 30000 //In Milliseconds

#define HEARTBEAT_KILL_TIMEOUT 20000 //In Milliseconds

#define UHF_KILL_TIMEOUT 10000

typedef enum _p_priority {
    PACKET_TYPE_POSITION = 0,
    PACKET_TYPE_STATUS = 1,
    PACKET_TYPE_GAINS = 2,
    PACKET_TYPE_CHANNELS = 3
} PacketType;


/* For reference: 
 In MPLAB XC 16 compiler:
 char           : 1 byte
 int            : 2 bytes
 long int       : 4 bytes
 float          : 4 bytes
 long double    : 8 bytes
 */

//62 bytes. High Frequency - Multiple times per second
struct packet_type_position_block { //
    long double lat, lon;
    uint32_t sysTime;
    float UTC;
    float pitch, roll, yaw;
    float pitchRate, rollRate, yawRate;
    float airspeed;
    float alt;
    float gSpeed;
    uint16_t heading;
    
};

//36 bytes. Medium frequency. About once every second
struct packet_type_status_block {
    int16_t rollRateSetpoint, pitchRateSetpoint, yawRateSetpoint; 
    int16_t rollSetpoint, pitchSetpoint;
    int16_t headingSetpoint, altitudeSetpoint, throttleSetpoint;
    
    int16_t batteryLevel1, batteryLevel2;
    uint16_t autonomousLevel;
    uint16_t startupErrorCodes; //2 bytes
    uint16_t dl_transmission_errors, ul_receive_errors;
    uint8_t ul_rssi, uhf_rssi, uhf_link_quality;
    uint8_t pathFollowing;
    uint8_t waypointIndex;
    uint8_t gpsStatus;
    uint8_t numWaypoints;
    uint8_t autopilotActive;
};

//80 bytes
struct packet_type_gain_block{
    float roll_rate_kp, roll_rate_kd, roll_rate_ki;
    float pitch_rate_kp, pitch_rate_kd, pitch_rate_ki;
    float yaw_rate_kp, yaw_rate_kd, yaw_rate_ki;
    float roll_angle_kp, roll_angle_kd, roll_angle_ki;
    float pitch_angle_kp, pitch_angle_kd, pitch_angle_ki;
    
    float heading_kp;
    float altitude_kp;
    float ground_speed_kp;
    
    float path_kp;
    float orbit_kp;
};

//32 bytes
struct packet_type_channels_block {
    int16_t ch1_in, ch2_in, ch3_in, ch4_in, ch5_in, ch6_in, ch7_in, ch8_in;
    int16_t ch1_out, ch2_out, ch3_out, ch4_out, ch5_out, ch6_out, ch7_out, ch8_out;
    bool channels_scaled; //whether the following values are scaled, or raw
};

typedef union {
    struct packet_type_position_block position_block;
    struct packet_type_status_block status_block;
    struct packet_type_gain_block gain_block;
    struct packet_type_channels_block channels_block;
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
TelemetryBlock* createTelemetryBlock(PacketType packet);

/**
 * Queues a telemetry block to be sent down the data link
 * @param data
 * @return True if successfully queued, false otherwise
 */
bool queueTelemetryBlock(TelemetryBlock* data);

#endif
