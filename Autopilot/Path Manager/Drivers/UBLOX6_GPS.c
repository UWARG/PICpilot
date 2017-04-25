/*
 * @file WARG_GPS.h
 * @author Serj Babayan
 * @created April 23, 2017, 2:07 PM
 */

#include "UBLOX6_GPS.h"
#include "../Utilities/NMEAParser.h"
#include "../../Common/Interfaces/UART.h"
#include "../Peripherals/GPS.h"
#include "../../Common/Utilities/Logger.h"
#include "../../Common/Clock/Timer.h"
#include <stdbool.h>
#include <string.h>
#include <xc.h>

#if USE_GPS == GPS_UBLOX_6

#define RECEIVE_BUFFER_LENGTH 100

//should be enough to hold any length of nmea string
static char receive_buffer[RECEIVE_BUFFER_LENGTH];
static uint8_t receive_buffer_index = 0;
static uint16_t communication_errors = 0;
static uint32_t last_receive_time = 0;
static bool data_available = false;

GPSData gps_data;

void initGPS(){
    //setup a 400 byte buffer for transmissions
    initUART(UBLOX6_UART_INTERFACE, UBLOX6_UART_BAUD_RATE, 400, 800, UART_TX_RX_ENABLE);
}

void requestGPSInfo(){
//    while(1){
//        while(getRXSize(2) != 0){
//            U1TXREG = readRXData(2);
//        }
//    }
    
    static bool currently_parsing_string = false;
    char data;
    
    while (getRXSize(UBLOX6_UART_INTERFACE) != 0){
        data = readRXData(UBLOX6_UART_INTERFACE);

        if (data == '$'){ //we've just started parsing a packet
            currently_parsing_string = true;
        } else if (data == '\n'){ //we've just finished parsing a packet
            currently_parsing_string = false;
            receive_buffer_index = 0;
            if (isValidNMEAString(receive_buffer, RECEIVE_BUFFER_LENGTH)){
                if (strncmp(receive_buffer, GGA_HEADER, 5) == 0){ //if we received a gga string
                    last_receive_time = getTime();
                    parseGGA(receive_buffer, &gps_data.latitude, &gps_data.longitude, &gps_data.utc_time, &gps_data.altitude, &gps_data.fix_status, &gps_data.num_satellites);
                    data_available = true;
                } else if (strncmp(receive_buffer, VTG_HEADER, 5) == 0){ //if we received a vtg string
                    last_receive_time = getTime();
                    parseVTG(receive_buffer, &gps_data.ground_speed, &gps_data.heading);
                    data_available = true;
                } //otherwise ignore other string types

                //we'll break out of the loop here as we dont want this operation to take too long
                //(we'll parse another packet, if available, in the next iteration)
                break;
                
            } else {
                communication_errors++;
            }
        } else if (currently_parsing_string){
            receive_buffer[receive_buffer_index] = data;
            receive_buffer_index = (receive_buffer_index + 1) % RECEIVE_BUFFER_LENGTH;
        }
    }
}

uint16_t getGPSCommunicationErrors(){
    return communication_errors;
}

bool isNewGPSDataAvailable(){
    if (data_available){
        data_available = false;
        return true;
    }
    return false;
}

bool isGPSConnected(){
    return (getTime() - last_receive_time) <= UBLOX6_DISCONNECT_TIMEOUT;
}

#endif