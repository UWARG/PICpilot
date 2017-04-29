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

/** Time in ms on how often to send down a packet. Packets will not be send to the radio faster than this */
#define DOWNLINK_SEND_INTERVAL 150

/** Time in miliseconds for how often to check for new messages from the uplink **/
#define UPLINK_CHECK_FREQUENCY 500

/**
 * Different packet types that we can send over via the downlink
 */
typedef enum {
    PACKET_TYPE_POSITION = 0,
    PACKET_TYPE_STATUS = 1,
    PACKET_TYPE_GAINS = 2,
    PACKET_TYPE_CHANNELS = 3
} PacketType;

/**
 * Defines the normal order that packets will be send down.  Note that this list contains packet types that should continually
 * be sent down. It should NOT contain packets types that should be sent down from
 * an event, ie gains. For these types of packets, the queuePacketType() method should
 * be called to move the packet type to the front of the queue
 */
static const uint8_t DEFAULT_PACKET_ORDER[] = {
    PACKET_TYPE_POSITION,
    PACKET_TYPE_POSITION,
    PACKET_TYPE_STATUS,
    PACKET_TYPE_POSITION,
    PACKET_TYPE_POSITION,
    PACKET_TYPE_CHANNELS
};

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
    uint32_t sys_time;
    float gps_time;
    float pitch, roll, yaw;
    float pitch_rate, roll_rate, yaw_rate;
    float airspeed;
    float altitude;
    float ground_speed;
    int16_t heading;
};

//50 bytes. Medium frequency. About once every second
struct packet_type_status_block {
    float path_checksum;
    int16_t roll_rate_setpoint, pitch_rate_setpoint, yaw_rate_setpoint; 
    int16_t roll_setpoint, pitch_setpoint;
    int16_t heading_setpoint, altitude_setpoint, throttle_setpoint;
    
    int16_t internal_battery_voltage, external_battery_voltage;

    uint16_t program_state; //state of autopilot. vehicle type, armed, unarmed, kill mode. Heartbeat status
    uint16_t autonomous_level; //rate, angle control sources, etc
    uint16_t startup_errors;
    uint16_t am_interchip_errors, pm_interchip_errors, gps_communication_errors; //error counts for dma communications
    uint16_t dl_transmission_errors, ul_receive_errors; //xbee specific transmission errors
    uint16_t peripheral_status; //sensor and radio statuses
    uint16_t uhf_channel_status; //which channels are connected and disconnected
    uint8_t ul_rssi, uhf_rssi, uhf_link_quality; //ul_rssi is for telemetry receival rssi

    uint8_t waypoint_index;
    uint8_t waypoint_count;
};

//92 bytes
struct packet_type_gain_block{
    float roll_rate_kp, roll_rate_kd, roll_rate_ki;
    float pitch_rate_kp, pitch_rate_kd, pitch_rate_ki;
    float yaw_rate_kp, yaw_rate_kd, yaw_rate_ki;
    float roll_angle_kp, roll_angle_kd, roll_angle_ki;
    float pitch_angle_kp, pitch_angle_kd, pitch_angle_ki;
    
    float heading_kp, heading_ki;
    float altitude_kp, altitude_ki;
    float ground_speed_kp, ground_speed_ki;
    
    float path_kp;
    float orbit_kp;
};

//34 bytes
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

typedef struct {
    uint16_t type; // 2 bytes as this is what the data relay expects the size to be
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
 * @return The packet type that should be sent down in the next transmission
 */
PacketType getNextPacketType(void);

/**
 * Call this method to request a specific packet type to be queued down in the downlink.
 * This call will make this packet type take priority over the continous/default packets
 * @param type
 */
void queuePacketType(PacketType type);

/**
 * Queues a telemetry block to be sent down the data link
 * @param data
 * @return True if successfully queued, false otherwise
 */
bool queueTelemetryBlock(TelemetryBlock* data);

#endif
