/**
 * @file Timer.h
 * @author Serj Babayan
 * @created April 23, 2017, 2:09AM
 *
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#include <stdint.h>
#include <xc.h>

static volatile uint32_t time_ms = 0;

/**
 * Initializes Timer1 as a 1ms, 16-bit timer
 */
void initTimer1(){
    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TCS = 0; // Select internal instruction cycle clock
    T1CONbits.TGATE = 0; // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b10; // Select 1:8 Prescaler
    TMR1 = 0x00; // Clear timer register
    PR1 = 250; // Load the period value. Trigger every ms
    IPC0bits.T1IP = 0x01; // Set Timer 4 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer 4 Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer 4 interrupt
    T1CONbits.TON = 1; // Start Timer
}

void delay(uint32_t ms){
    uint32_t start_time = time_ms;
    while(time_ms - start_time < ms);
}

uint32_t getTime(){
    return time_ms;
}

/**
 * Timer1 interrupt. Executed every ms
 */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void){
    time_ms += 1;
    IFS0bits.T1IF = 0;
}
