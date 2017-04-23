#include "UART.h"
#include "Logger.h"
#include "Timer.h"
#include "SPI.h"
#include "Peripherals.h"
#include "p24F16KA101.h"
#include <xc.h>
#include <math.h>
#include "Config.h"
#include "GPS.h"
#include <stdint.h>

void transmitData(void) {
//    int i;
//    char *dataGPS = &gpsData;
//    char spiChecksum = 0;
//
//
//    PORTBbits.RB15 = 0; //slave enabled
//    for (i = 0; i < sizeof(struct GPSData); i++)
//    {
//
//        while (SPI1STATbits.SPITBF != 0) {;}
//        SPI1BUF = dataGPS[0];
////        spiChecksum ^= dataGPS[0];
//        dataGPS++;
//    }
////    while (SPI1STATbits.SPITBF != 0) {;}
////    SPI1BUF = spiChecksum;
//    //Delay_half_us(110);
//    PORTBbits.RB15 = 1; //slave disabled

}
uint32_t start_time = 0;
bool toggle = true;
int main(void) {
    initTimer1();

    delay(200); //wait a bit to make sure we're getting consistent power
    
    initLogger();
    initLED();
//    initSPI(); //initialize SPI bus
    initGPS();
    debug("Letting GPS module start up for 2 sec..");
    setLED(1);
    delay(2000);
    
    
    configureGPS(); //configure gps if battery was removed
    
    while (1) {
        parseIncomingGPSData();
        if(isNewDataAvailable()){
            debug("new data available!");
        }

    }
}






