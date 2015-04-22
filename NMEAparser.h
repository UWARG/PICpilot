#ifndef NMEAPARSER_H
#define	NMEAPARSER_H

#include "InterchipDMA.h"

#if PATH_MANAGER && !GPS_OLD
double convertLatLong(char latOrLong[]);
//void parseGLL(char data[], GPSData *GPSData);
void parseGGA(char data[], GPSData *GData);
//void parseTHS(char data[], GPSData *GData);
void parseVTG(char data[], GPSData *GData);

void assembleNEMAMessage();
#endif



#endif	/* NMEAPARSER_H */

