/**
 * @file InterchipDMA.c
 * @author Chris Hajduk, Serj Babayan
 * @created February 2, 2014, 2:10 PM
 * @description
 * This file manages the initialization of the interchip DMA channel that piggy backs
 * off of the SPI1 interface. Default operation mode is one-shot mode. The path manager
 * is the master and attitude manager is the slave in the SPI bus.
 * 
 * DMA channel 0 is designated to receival of SPI messages, and DMA channel 1 is
 * designated to sending.
 * 
 * Note that also the Path Manager is responsible for initiating the DMA writes as
 * this is in one shot mode. It should also perform a write every certain interval
 * of time to ensure that it can read any relevant attitude manager data, such as
 * what waypoints to add.
 * 
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef INTERCHIPDMA_H
#define	INTERCHIPDMA_H

#include "../Common.h"

#define DMA_CHIP_ID_PATH_MANAGER 0
#define DMA_CHIP_ID_ATTITUDE_MANAGER 1

/**
 * SPI port that interchip communications happen
 */
#define IC_DMA_PORT 1

/**
 * SPI frequency of the channel in khz
 */
#define DMA_CLOCK_KHZ 40000 //40Mhz

/**
 * Data that the path manager sends to the attitude manager. Note that its
 * vital that the last byte be the checksum byte!
 */
typedef struct { // 60 Bytes
    long double latitude;  // 8 Bytes - ddd.mmmmmm
    long double longitude; // 8 Bytes - ddd.mmmmmm
    float time;     // 4 Bytes   -  hhmmss.ssss
    float speed;    //KM/H
    float altitude;
    float airspeed;
    float pmPathGain;
    float pmOrbitGain;
    float waypointChecksum;
    int sp_Altitude; // Meters
    int heading;  //Degrees
    int sp_Heading; //Degrees
    int batteryLevel1;
    int batteryLevel2;
    char satellites;    //1 Byte
    char positionFix;   //0 = No GPS, 1 = GPS fix, 2 = DGSP Fix
    char targetWaypoint;
    char waypointCount;
    char pathFollowing;
} PMData;

/**
 * Data that the attitude manager sends to the path manager. Note its vital
 * that the last byte be the checksum byte!
 */
typedef struct {
    WaypointWrapper waypoint;
    float pathGain;
    float orbitGain;
    float calibrationHeight;
    char command;
    char followPath;
} AMData;

typedef union{
    PMData pm_data;
    AMData am_data;
} DMADataBuffer;

/**
 * Global send buffer. Attitude manager should write to the am_data field,
 * path manager should write to the pm_data field
 */
extern volatile DMADataBuffer interchip_send_buffer;

/**
 * Global receive buffer. Attitude manager should read the pm_data field, path
 * manager should read the am_data field. Note that reads should only happen if
 * the isDMADataAvailable function returns true!
 */
extern volatile DMADataBuffer interchip_receive_buffer;

/**
 * Initializes all the DMA channels
 * @param chip_id Which chip is currently running the code. Differentiates between
 *      path and attitude manager
 */
void initInterchip(uint8_t chip_id);

/** 
 * Whether there is data that can be read from the buffer. Note that calling this
 * function will reset the flag!
 */
bool newInterchipData(void);

/**
 * Triggers a send of the DMA buffer. In the case of the path manager, will
 * copy over the data to the data buffer, and force a DMA update. In the case
 * of the attitude manager, will simply copy over the data. The next path manager
 * update will send it
 */
void sendInterchipData(void);

/**
 * @return Total number of communication errors between the path and
 * attitude manager
 */
uint16_t getInterchipErrorCount(void);

#endif

