/*
 * @file WARG_GPS.h
 * @author Serj Babayan
 * @created April 23, 2017, 2:07 PM
 */

#include <stdbool.h>
#include "../../Common/Interfaces/UART.h"
#include "UBLOX6_GPS.h"
#include "../Peripherals/GPS.h"
#include "../../Common/Utilities/Logger.h"
#include <string.h>
#include <xc.h>

#if USE_GPS == GPS_UBLOX_6

GPSData gps_data;

void initGPS(){
    //setup a 400 byte buffer for transmissions
    initUART(UBLOX6_UART_INTERFACE, UBLOX6_UART_BAUD_RATE, 400, 800, UART_TX_RX_ENABLE);
}

void requestGPSInfo(){
    
}

uint16_t getGPSCommunicationErrors(){
    return 1;
}

bool isNewGPSDataAvailable(){
   return true;
}

bool isGPSConnected(){
    return true;
}

#endif