/**
 * @file MTK33x9.h
 * Contains implementation for parsing GPS
 */
#include "UART.h"
#include "MTK33x9.h"
#include <stdbool.h>

static bool data_available = false;

static void sendCommand(const char* msg);

void initGPS(){
    initUART(GPS_UART_INTERFACE, GPS_UART_BAUDRATE);
}

void configureGPS(){
    debug("Setting GPS to output only desired NMEA sentences");
    sendCommand(PMTK_SET_NMEA_OUTPUT_GGAVTG);
    delay(300);
    
    debug("Setting GPS NMEA update rate to 10Hz");
    sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
    delay(300);
    
    debug("Setting GPS positional update rate to 5Hz");
    sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    delay(300);

    debug("Setting GPS to use WAAS (DGPS)");
    sendCommand(PMTK_ENABLE_WAAS);
    delay(300);

    debug("GPS has been configured");
};

void isNewDataAvailable(){
    return data_available;
}

static void sendCommand(const char* msg){
    uint16_t length = 0;
    uint16_t i = 0;
    while(msg[i] != 0){
        length++;
    }
    //send the command via uart
    sendTXData(GPS_UART_INTERFACE, msg, length);
}