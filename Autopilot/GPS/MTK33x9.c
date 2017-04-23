/**
 * @file MTK33x9.h
 * Contains implementation for parsing GPS
 */
#include "UART.h"
#include "GPS.h"
#include "Logger.h"
#include "Timer.h"
#include "Utilities.h"
#include "MTK33x9.h"
#include "p24F16KA101.h"
#include <stdbool.h>
#include <string.h>

GPSData gps_data;

static bool data_available = false;
static bool new_vtg_data = false;
static bool new_gga_data = false;

static char gga_buffer[GPS_UART_BUFFER_SIZE]; //buffer for pasing gga (positional packets)
static char vtg_buffer[GPS_UART_BUFFER_SIZE]; //buffer for parsing vtg packets (velocity packets)
static char uart_buffer[GPS_UART_BUFFER_SIZE]; //buffer for parsing vtg packets (velocity packets)

static void sendCommand(const char* msg);
static bool isNMEAChecksumValid(char* string);
static void parseGGA(char* data);
static void parseVTG(char* data);

void initGPS(){
    initUART(GPS_UART_INTERFACE, GPS_UART_BAUDRATE);
}

void configureGPS(){
    debug("Setting GPS to output only desired NMEA sentences");
    sendCommand(PMTK_SET_NMEA_OUTPUT_GGAVTG);
    debug("done");
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

bool isNewDataAvailable(){
    return data_available;
}

void parseIncomingGPSData(){
    if (!new_gga_data && !new_vtg_data){ //if no data has been copied over
        return;
    }

    if (new_gga_data){
        new_gga_data = false;
        if (isNMEAChecksumValid(gga_buffer)){
            data_available = false;
            parseGGA(gga_buffer);
            data_available = true;
        }
    }

    if (new_vtg_data){
        new_vtg_data = false;
        if (isNMEAChecksumValid(vtg_buffer)){
            data_available = false;
            parseVTG(gga_buffer);
            data_available = true;
        }
    }
}

static void sendCommand(const char* msg){
    uint16_t i = 0;
    while(msg[i] != 0){
        i++;
    }
    //send the command via uart
    sendTXData(GPS_UART_INTERFACE, (uint8_t*)msg, i);
}

/**
 * Called when we get RX data from the GPS module
 */
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
    static bool currently_parsing = false;
    static uint16_t buffer_index = 0;
    static uint8_t data;

    while (U1STAbits.URXDA) { //while the receive register has data available
        data = U1RXREG;
        
        if (data == '$') { //Beginning of Packet
            currently_parsing = true;
        } else if (data == 0x0A) { //End of Packet
             if (strncmp(GPS_GGA_MESSAGE, uart_buffer, 6) == 0){
                 memcpy(gga_buffer, uart_buffer, GPS_UART_BUFFER_SIZE);
                 new_gga_data = true;
             } else if (strncmp(GPS_VTG_MESSAGE, uart_buffer, 6) == 0){
                memcpy(vtg_buffer, uart_buffer, GPS_UART_BUFFER_SIZE);
                new_vtg_data = true;
             }
             currently_parsing = false;
        } else if (currently_parsing){
            uart_buffer[buffer_index] = data;
            buffer_index = (buffer_index + 1) % GPS_UART_BUFFER_SIZE; //make sure we dont cause a memory fault here
        }
    }
    IFS0bits.U1RXIF = 0; // Clear the Interrupt Flag
}

/**
 * Given an NMEA string starting after the $, verifies the integrity of the stirng
 * using the checksum
 * @param string
 * @return True if string is a valid gps string, false otherwise
 */
static bool isNMEAChecksumValid(char* string){
    uint16_t i = 0;
    uint8_t checksum = 0;

    while(string[i] != '*'){
        if (string[i] != ','){
            checksum ^= string[i];
        }
        i++;
    }
    i++;
    
    if (byteToHexString((checksum & 0xF0) >> 4) == string[i] && byteToHexString(checksum & 0x0F) == string[i+1]){
        return 1;
    }
    return 0;
}

static void parseVTG(char* data){
    //static so that we dont allocate these variables every time
    static char rawHeading[6] = {0, 0, 0, 0, 0, 0};
    static char rawGroundSpeed[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    
    int comma = 0;
    int i = 0;
    int j = 6;

    while (data[j] != '*') {
        char numData = asciiToHex(data[j]);
        if (data[j] == ',') {
            comma++;
            i = 0;
        }
        if (comma == 1 && (i != 0)) {
            rawHeading[i] = numData;
        }
        if (comma == 7 && (i != 0)) {
            rawGroundSpeed[i] = numData;
        }
        i++;
        j++;
    }

    i = 1;
    long int multiplier = 10;
    int decimalPoint = 0;
    
    gps_data.heading = 0;
    float tHeading = 0;
    for (i = 1; i < 6; i++) //this code first generates an 5 digit decimal number
    {
        if (rawHeading[i] == 0x10)//check for decimal point
        {
            decimalPoint = i;
        } else {
            tHeading += (float) (rawHeading[i]*100000 / multiplier);
            multiplier *= 10;
        }
    }
    decimalPoint = decimalPoint - 2;
    multiplier = 10000;
    while (decimalPoint > 0) //then divdes it according to the placement of the decimal
    {
        multiplier = multiplier / 10;
        decimalPoint--;
    }
    gps_data.heading = (int)(tHeading / multiplier);

    //	//calculate speed - tricky because of unknown 1-3 digits preceeding the decimal
    i = 1;
    multiplier = 10;
    decimalPoint = 0;
    gps_data.ground_speed = 0;
    for (i = 1; i < 7; i++) //this code first generates an 6 digit decimal number
    {
        if (rawGroundSpeed[i] == 0x10)//check for decimal point
        {
            decimalPoint = i;
        } else {
            gps_data.ground_speed += (float) (rawGroundSpeed[i]*1000000 / multiplier);
            multiplier = multiplier * 10;
        }
    }
    decimalPoint = decimalPoint - 2;
    multiplier = 100000;
    while (decimalPoint > 0) //then divdes it according to the placement of the decimal
    {
        multiplier = multiplier / 10;
        decimalPoint--;
    }
    gps_data.ground_speed = gps_data.ground_speed / multiplier;
}

/**
 * Parses a GGA type NEMA string and modifies the GPS data struct with the applicable
 * fields
 * @param data
 */
static void parseGGA(char* data){
    int comma = 0; //comma counting so that we know what header we're parsing for
    int i = 0; //index for the current position of the field value
    int j = 6; //7th character is where data will start. index for the byte index in the counter

    //statically initialize placeholders for values. Static to avoid reinitializations every time
    static char rawTime[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    static char rawLatitude[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    static char rawLongitude[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    static char rawSatellites[3] = {0, 0, 10};
    static char rawAltitude[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    static char latitudeNS = 0;
    static char longitudeEW = 0;
    static char positionFix = 0;

    while (data[j] != '*') {
        char numData = asciiToHex(data[j]);

        if (data[j] == ',') {
            comma++;
            i = 0;
        }
        if ((comma == 1) && (i != 0)) {
            rawTime[i] = numData;
        }
        if ((comma == 2) && (i != 0)) {
            rawLatitude[i] = numData;

        }
        if ((comma == 3) && (i != 0)) {
            latitudeNS = data[j];

        }
        if ((comma == 4) && (i != 0)) {
            rawLongitude[i] = numData;

        }
        if ((comma == 5) && (i != 0)) {
            longitudeEW = data[j];

        }
        if ((comma == 6) && (i != 0)) {
            positionFix = numData;

        }
        if ((comma == 7) && (i != 0)) {
            rawSatellites[i] = numData;
        }
        if ((comma == 9) && (i != 0)) {
            rawAltitude[i] = numData;
        }
        i++;
        j++;
    }

    //now we've got all the valid data placed in our buffers. Modify gps data struct to match

    //calculate time
    gps_data.utc_time = (float) rawTime[1] * 100000;
    gps_data.utc_time += (float) rawTime[2] * 10000;
    gps_data.utc_time += (float) rawTime[3] * 1000;
    gps_data.utc_time += (float) rawTime[4] * 100;
    gps_data.utc_time += (float) rawTime[5] * 10;
    gps_data.utc_time += (float) rawTime[6] * 1;
    //Decimal Point
    gps_data.utc_time += (float) rawTime[8] * 0.1;
    gps_data.utc_time += (float) rawTime[9] * 0.01;
    gps_data.utc_time += (float) rawTime[10] * 0.001;

    //calculate latitude
    gps_data.latitude = rawLatitude[3]*10.0;
    gps_data.latitude += rawLatitude[4]*1.0;
    gps_data.latitude += rawLatitude[6]*0.1;
    gps_data.latitude += rawLatitude[7]*0.01;
    gps_data.latitude += rawLatitude[8]*0.001;
    gps_data.latitude += rawLatitude[9]*0.0001;
    gps_data.latitude /= 60;  //Converts from dd.mmmmmm to decimal degrees. (60 minutes in a degree)
    //Then add the degrees (ranges from -90 to +90)
    gps_data.latitude += rawLatitude[1]*10.0;
    gps_data.latitude += rawLatitude[2]*1.0;

    if (latitudeNS == 'S'){
        gps_data.latitude *= -1;
    }

    //calculate longitude
    gps_data.longitude = rawLongitude[4]*10.0;
    gps_data.longitude += rawLongitude[5]*1.0;
    gps_data.longitude += rawLongitude[7]*0.1;
    gps_data.longitude += rawLongitude[8]*0.01;
    gps_data.longitude += rawLongitude[9]*0.001;
    gps_data.longitude += rawLongitude[10]*0.0001;
    gps_data.longitude /= 60;  //Converts from ddd.mmmmmm to decimal degrees. (60 minutes in a degree)
    //Then add the degrees (ranges from -180 to +180)
    gps_data.longitude += rawLongitude[1]*100.0;
    gps_data.longitude += rawLongitude[2]*10.0;
    gps_data.longitude += rawLongitude[3]*1.0;

    if (longitudeEW == 'W'){
        gps_data.longitude *= -1;
    }

    //calculate satellites
    if (rawSatellites[2] == 10) gps_data.num_satellites = rawSatellites[1];
    else gps_data.num_satellites = rawSatellites[1]*10 + rawSatellites[2];

    //calculate altitude - tricky because of unknown 1-3 digits preceeding the decimal
    i = 1;
    long int multiplier = 10;
    int decimalPoint = 0;
    gps_data.altitude = 0;
    float tAltitude = 0;
    for (i = 1; i < 8; i++) //this code first generates an 6 digit decimal number
    {
        if (rawAltitude[i] == 0x10) //check for decimal point
        {
            decimalPoint = i;
        } else {
            tAltitude += (float) (rawAltitude[i]*1000000 / multiplier);
            multiplier *= 10;
        }
    }
    decimalPoint = decimalPoint - 2;
    multiplier = 100000;
    while (decimalPoint > 0) //then divides it according to the placement of the decimal
    {
        multiplier = multiplier / 10;
        decimalPoint--;
    }
    gps_data.altitude = (int)(tAltitude / multiplier);

}