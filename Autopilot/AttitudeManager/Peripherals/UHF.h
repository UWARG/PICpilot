/* 
 * @file UHF.h
 * @author Serj Babayan
 * @created March 26, 2017, 9:00 PM
 * UHF peripheral file for retrieving uhf connection statistics such as RSSI
 * and signal quality. Parses the appropriate PWM channels inputs and converts
 * them to percentages for sending down the downlink
 */

#ifndef UHF_H
#define	UHF_H

#include <stdint.h>

#define UHF_RSSI_CHANNEL 6

#define UHF_LINK_QUALITY_CHANNEL 7

/**
 * Note that we're not going to be using
 * @param sys_time
 * @return 
 */
#define UHF_RSSI_MAX_PWM 1210
#define UHF_RSSI_MIN_PWM 625
#define UHF_LINK_QUALITY_MAX_PWM 1170
#define UHF_LINK_QUALITY_MIN_PWM 625

/**
 * @param sys_time Current system time in ms
 * @return A percentage value from 0-100 of what the relative signal strength
 * of the uplink uhf connection is
 */
uint8_t getUHFRSSI(uint32_t sys_time);

/**
 * @param sys_time Current system time in ms
 * @return A percentage value from 0-100 representing the signal quality. Different
 * radios represent this differently. This may be a percentage of the frames dropped
 * out of the last 15 frames in the case of the orange rx, or something else.
 */
uint8_t getUHFLinkQuality(uint32_t sys_time);

#endif

