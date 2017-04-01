/* 
 * @file UHF.h
 * @author Serj Babayan
 * @created March 26, 2017, 9:00 PM
 */
#include "../InputCapture.h"
#include "../PWM.h"
#include "UHF.h"

uint8_t getUHFRSSI(uint32_t sys_time){
    if (getPWMInputStatus() & UHF_RSSI_CHANNEL == 1){
        return 0;
    }
    uint16_t unscaled = getICValues(sys_time)[UHF_RSSI_CHANNEL - 1];
    int16_t scaled = (((unscaled - LOWER_PWM)*100)/(UPPER_PWM - LOWER_PWM));
    if (scaled > 100){
        return 100;
    } else if (scaled < 0){
        return 0;
    }
    return (uint8_t)scaled;
}

uint8_t getUHFLinkQuality(uint32_t sys_time){
    if (getPWMInputStatus() & UHF_LINK_QUALITY_CHANNEL == 1){
        return 0;
    }
    uint16_t unscaled = getICValues(sys_time)[UHF_LINK_QUALITY_CHANNEL - 1];
    int16_t scaled = (((unscaled - LOWER_PWM)*100)/(UPPER_PWM - LOWER_PWM));
    if (scaled > 100){
        return 100;
    } else if (scaled < 0){
        return 0;
    }
    return (uint8_t)scaled;
}