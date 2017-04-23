#include "ClockConfig.h"
#include "Config.h"
#include "UART.h"
#include "Logger.h"
#include "Timer.h"
#include "SPI.h"
#include "Peripherals.h"
#include "GPS.h"
#include <stdint.h>
#include <xc.h>
#include <math.h>

uint32_t led_timer = 0;
int main(void) {
    uint32_t sys_time = 0;
    
    initTimer1();

    delay(200); //wait a bit to make sure we're getting consistent power
    
    initLogger();
    initLED();
    initSPI(SPI_MODE); //initialize SPI bus
    initGPS();
    setLED(1);
    debug("Letting GPS module start up for 1 sec..");
    delay(1000);
    
    configureGPS(); //configure gps
    
    while (1) {
        sys_time = getTime();
        
        parseIncomingGPSData();
        if(isNewDataAvailable()){ //if we received new information from the gps
            led_timer = sys_time;
            queueTransmitData();
        }

        //Behavior: LED will flash if it successfully receiving and parsing valid NMEA packets from the gps module
        if (sys_time - led_timer < 50){
            setLED(1);
        } else {
            setLED(0);
        }
    }
}
