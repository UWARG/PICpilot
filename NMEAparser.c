#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include "UART1.h"
#include "UART2.h"
#include "NMEAparser.h"
//#include "InterchipDMA.c"
//#include <libpic30.h>


/* This is all the information we can get.
GGA:
- latitude
- longitude
- position fix
- satellite used 0 to 12
GST:
- std deviations
THS:
- heading in degrees
RMC:
- speed over ground
VTG:
- speed over ground in knots and kmh
- fixed field (kmh)
- course over ground (degrees)
-
*/

#if PATH_MANAGER && !GPS_OLD

GPSData gpsData;
extern UART_RX_Buffer _buff;
char nMsg[255];
char dmsg[] = "$GPGGA,092725.00,4717.11399,N,00833.91590,W,1,8,1.01,499.6,M,48.0,M,,0*5B";
short nPos = 0;

// Convert Lat and Long to degrees for GLL
double convertLatLong(char latOrLong[]) {
	// converts coordinate to double, then truncates, then retrieves decimal value by /100
        double input = atof(latOrLong)/100;
        double minutes = ((input - (int)input)/60)*100;
	// Return Decimal value + fractional value of coordinate.
	return ((int)input + minutes);
}

//GLL
//void parseGLL(char data[], GPSData *GPSData) {
//
//	char values[8][12] = {0}; //need to make a clear unused memory function? Or make dynamically allocated array...
//	int i = 0; //increment in for loop (used to loop through 8 parameters sent to function)
//	int j = 0; //increment in while loop (used to loop through each char in each element in data)
//	int n = 0; //increment in while loop (used to assign each char in data to values)
//
//	// fills the values array with information from the data[] variable
//	for (i = 0; i < 8; i++) {
//		n = 0;
//		while (data[j] != ',' && data[j] != '\0') {
//			values[i][n] = data[j];
//			j++;
//			n++;
//		}
//		j++;
//	}
//
//	// print all values to check
////	int k = 0;
////	for (k = 0; k < 8; k++) {
////		printf("%s : %s\n", names[k], values[k]);
////	}
//
////	printf("\n");
//
//	// fill the GPSData struct with info from values[]
//
//	GPSData->latitude = convertLatLong(values[1]);
//        if(values[2][1] == 'S') GPSData->latitude *= -1;
//	GPSData->longitude = convertLatLong(values[3]);
//        if(values[4][1] == 'W') GPSData->longitude *= -1;
//	sscanf(values[5], "%f", &GPSData->time);
//
//        UART1_SendString(data);
//        UART1_SendString("\n\n");
//        UART1_SendString(values[4]);
//        UART1_SendString("\n\n");
//        char buffer[100];
//	sprintf(buffer,"Latitude in GPS object: %f\n", GPSData->latitude);
//        UART1_SendString(buffer);
//        sprintf(buffer,"Longitude in GPS object: %f\n", GPSData->longitude);
//        UART1_SendString(buffer);
//	printf("Time in GPS Object: %f\n", GPSData->time);
//
//}

// GGA 
void parseGGA(char data[], GPSData *GData) {
	char values[15][12] = {0}; //need to make a clear unused memory function? Or make dynamically allocated array...
	int i = 0; //increment in for loop (used to loop through 8 parameters sent to function)
	int j = 0; //increment in while loop (used to loop through each char in each element in data)
	int n = 0; //increment in while loop (used to assign each char in data to values)

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

        // fill the GPSData struct with info from values[]
	GData->latitude = convertLatLong(values[2]);
        if(values[3][0] == 'S') GData->latitude *= -1;
	GData->longitude = convertLatLong(values[4]);
        if(values[5][0] == 'W') GData->longitude *= -1;
        sscanf(values[1], "%f", &GData->time);

	sscanf(values[9], "%d", &GData->altitude);
        sscanf(values[6], "%c", &GData->positionFix); //0 is no fix, 1 or 2 is valid fix, 6 is dead reckoning
        sscanf(values[8], "%c", &GData->satellites); //Space Vehicles used, value between 0 and 12

//        char buffer[100];
//	sprintf(buffer,"Latitude: %f\n", GData->latitude);
//        UART1_SendString(buffer);
//        sprintf(buffer,"Longitude: %f\n", GData->longitude);
//        UART1_SendString(buffer);
//	sprintf(buffer, "Time: %f\n", GData->time);
//        UART1_SendString(buffer);
//	sprintf(buffer, "Altitude: %d\n", GData->altitude);
//        UART1_SendString(buffer);
//        sprintf(buffer, "Pos Fix: %c\n", GData->positionFix);
//        UART1_SendString(buffer);
//        sprintf(buffer, "Satellites: %c\n", GData->satellites);
//        UART1_SendString(buffer);

//        char buffer[100];
//	sprintf(buffer,"altitude in GPS object: %d\n", GData->altitude);
//        UART1_SendString(buffer);
}

//VTG
void parseVTG(char data[], GPSData *GData) {

	char values[10][7] = {0}; //need to make a clear unused memory function? Or make dynamically allocated array...
	int i = 0; //increment in for loop (used to loop through 8 parameters sent to function)
	int j = 0; //increment in while loop (used to loop through each char in each element in data)
	int n = 0; //increment in while loop (used to assign each char in data to values)

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

        char buffer[100];
//	sprintf(buffer,"Speed: %f\n", GData->speed);
//        UART1_SendString(buffer);
        sprintf(buffer,"Course: %d\n", GData->heading);
        UART1_SendString(buffer);
}

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
               //if (nMsg[5] == 'L') parseGLL(nMsg,&gpsData);
               if (nMsg[5] == 'A') parseGGA(nMsg,&gpsData);
               if (nMsg[5] == 'G') parseVTG(nMsg,&gpsData);
               //if (nMsg[5] == 'S') parseTHS(nMsg,&gpsData);
               //newGPSData= true;
           }
           nPos = 0;
       }
    }  while(1);
}

#endif