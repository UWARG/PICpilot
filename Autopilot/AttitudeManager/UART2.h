/**
 * @file UART2.h
 * @author Mitch Hatfield
 * @date Jun 2013
 * @brief Settings for UART2 communication used for telemetry link 
 * @copyright Waterloo Aerial Robotics Group 2016 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#pragma once

#include "main.h"

/** Initializes the UART2 connection **/
void InitUART2();

/** UART2 interrupt function routine for when data comes in from the XBEE **/
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt( void );
