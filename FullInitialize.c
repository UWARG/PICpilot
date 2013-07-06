
//Include Header Files
#include <p33FJ256GP710A.h>
#include "Clock.h"
#include "UART2.h"
#include "lcd.h"

void FullInit()
{
    /* set LEDs (D3-D10/RA0-RA7) drive state low */
    LATA = 0xFF00;
    /* set LED pins (D3-D10/RA0-RA7) as outputs */
    TRISA = 0xFF00;

    InitClock();
    InitUART2();
    Init_LCD();

}
