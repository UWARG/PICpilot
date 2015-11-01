//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <float.h>
//#include <math.h>
//#include "UART1.h"
#include "UART2.h"
#include "NMEAparser.h"
#include "InterchipDMA.h"

#if PATH_MANAGER && !GPS_OLD
GPSData gpsData;
extern UART_RX_Buffer _buff;
extern char newGPSDataAvailable;

char nMsg[255];
short nPos = 0;

// This function converts the NMEA formatted latitude or longitude string of
// the format "DDMM.MMMMMM" (D = degrees, M = minutes) to simply degrees in
// decimal form. The result is returned as a double. This function ignores
// the global quadrant and thus all positions are > 0. The quadrant (sign) of
// the coodrinate is parsed separately.

double convertLatLong(char latOrLong[]) {

        //convert input string to double, divide by 100 to separate minutes from
        //degrees.
        double input = atof(latOrLong)/100;

        //convert the "minutes" portion of the above conversion to a decimal
        double minutes = ((input - (int)input)/60)*100;

	// Return degree value + fractional value of coordinate.
	return ((int)input + minutes);
}

// This message parses the "GGA" type NMEA message. This message contains
// the following information used to populate the GPSData struct:
// - latitude
// - longitude
// - time
// - altitude
// - position fix
// - satellites
// Message structure:
//$GPGGA,hhmmss.ss,Latitude,N,Longitude,E,FS,NoSV,HDOP,msl,m,Altref,m,DiffAge,DiffStation*cs<CR><LF>
//for complete message details see
//http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_%28GPS.G6-SW-10018%29.pdf

void parseGGA(char data[], GPSData *GData) {
	char values[15][12] = {0}; //store the value of the 15 components of this message (overkill, only 8 are needed)
	int i = 0;
	int j = 0;
	int n = 0;

	// fills the values array with information from the data[] variable
	for (i = 0; i < 15; i++) {
		n = 0;
		while (data[j] != ',' && data[j] != '\0') {
			values[i][n] = data[j];
			j++;
			n++;
		}
		j++;
	}
	// fill the GPSData struct with info from values[]
	GData->latitude = convertLatLong(values[2]);
        if(values[3][0] == 'S') GData->latitude *= -1;
	GData->longitude = convertLatLong(values[4]);
        if(values[5][0] == 'W') GData->longitude *= -1;
        sscanf(values[1], "%f", &GData->time);

	sscanf(values[9], "%d", &GData->altitude);
        sscanf(values[6], "%c", &GData->positionFix); //0 is no fix, 1 or 2 is valid fix, 6 is dead reckoning
       
        int temp;
        sscanf(values[7], "%d", &temp);//&GData->satellites); //Space Vehicles used, value between 0 and 12
        GData->satellites = (char)temp;
}

// This message parses the "VTG" type NMEA message. This message contains
// the following information used to populate the GPSData struct:
// - heading
// - speed
// Message structure:
// $GPVTG,cogt,T,cogm,M,sog,N,kph,K,mode*cs<CR><LF>
//for complete message details see
//http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_%28GPS.G6-SW-10018%29.pdf

void parseVTG(char data[], GPSData *GData) {

	char values[10][7] = {0}; //store the value of the 10 components of this message (overkill, only 2 are needed)
	int i = 0;
	int j = 0;
	int n = 0;

	// fills the values array with information from the data[] variable
	for (i = 0; i < 10; i++) {
		n = 0;
		while (data[j] != ',' && data[j] != '\0') {
			values[i][n] = data[j];
			j++;
			n++;
		}
		j++;
	}

	// fill the GPSData struct with info from values[]
	sscanf(values[7], "%f", &GData->speed);
        if(values[1] != 0) sscanf(values[1], "%d", &GData->heading);

	GData->speed = GData->speed / 3.6;
}

// This function takes characters from the UART2 RX buffer as they become
// and assembles them into complete NMEA messages. All NMEA messages start with
// a '$' character and end with a <CR> and <LF>. Once it has assembled a
// complete NMEA message, it calls the appropriate parser based on the
// type of message.
void assembleNEMAMessage()
{
    char c;
    do
    {
       c = read_rx_buffer(&_buff);
       if(c == '$')
           nPos = 0;
       else if (c == 0)
           return;

       nMsg[nPos] = c;
       nPos++;

       if(c == 10)
       {
           if(nMsg[0] == '$')
           {
               nMsg[nPos] = 0;
               if (nMsg[5] == 'A') parseGGA(nMsg,&gpsData);
               if (nMsg[5] == 'G') parseVTG(nMsg,&gpsData);
               newGPSDataAvailable = 1;
           }
           nPos = 0;
       }
    }  while(1);
}

#endif