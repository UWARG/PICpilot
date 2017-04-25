/**
 * @file LED.c
 * @author Serj Babayan
 * @created April 21, 2017, 12:43 AM
 *
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#include <xc.h>
#include <stdbool.h>

void initLED(){
    TRISAbits.TRISA6 = 0; //set A6 as a digital output
    PORTAbits.RA6 = 0; //turn the led off for now
}

void setLED(bool on){
    if (on){
        PORTAbits.RA6 = 1;
    } else {
        PORTAbits.RA6 = 0;
    }
}