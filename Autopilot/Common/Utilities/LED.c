#include "LED.h"

#define AM_LED (PORTGbits.RG15)
#define PM_LED (PORTDbits.RD1)

static int8_t chip_id = -1;

void initLED(bool isAttMan) {
    if (isAttMan){
        TRISGbits.TRISG15 = 0; // set RG15 to output (led pin)
        chip_id = 0;
    } else {
        TRISDbits.TRISD1 = 0; // set RD1/OC2 to an output (led pin)
        
        T3CONbits.TON = 0; // Disable Timer
        T3CONbits.TCS = 0; // Select internal instruction cycle clock
        T3CONbits.TGATE = 0; // Disable Gated Timer mode
        T3CONbits.TCKPS = 0x02; //1:64 scaler
        TMR3 = 0x00; // Clear timer register
        PR3 = 5*625; //set the period (5 ms)
        IPC2bits.T3IP = 0x01; // Set Timer 3 Interrupt Priority Level - Lowest
        IFS0bits.T3IF = 0; // Clear Timer 3 Interrupt Flag
        IEC0bits.T3IE = 0; // Disable Timer 3 interrupt
        T3CONbits.TON = 1; // Start Timer
        
        OC2CONbits.OCM = 0b000; // configure Output Compare module 2
        OC2RS = 0x00;
        OC2CONbits.OCTSEL = 1; // use timer 3
        OC2CONbits.OCM = 0b110;
        chip_id = 1;
    }
}

void setLEDState(bool on) {
    if (chip_id == 0) AM_LED = on; 
    else if (chip_id == 1) PM_LED = on;
}

void toggleLEDState() {
    if (chip_id == 0) AM_LED ^= 1; 
    else if (chip_id == 1) PM_LED ^= 1;
}

// only available for PM chip
// value: 0-255
void setLEDBrightness(uint8_t value) {
    if (chip_id != 1) return;
    OC2RS = (uint16_t)value * (PR3 / 255);
}
