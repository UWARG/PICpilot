#include "p24F16KA101.h"
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