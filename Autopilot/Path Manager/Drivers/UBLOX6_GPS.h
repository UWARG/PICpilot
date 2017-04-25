/*
 * @file UBLOX6_GPS.h
 * @author Serj Babayan
 * @created April 24, 2017
 * Driver for the ublox NEO-6M GPS module. Module is setup for NMEA string parsing
 */

#ifndef UBLOX6_H
#define	UBLOX6_H

#define UBLOX6_UART_INTERFACE 2

#define UBLOX6_UART_BAUD_RATE 9600

/**
 * Number of invalid messages we have to recieve from the GPS before we consider it
 * disconnected
 */
#define WUBLOX6_DISCONNECT_COM_COUNT 5

#endif

