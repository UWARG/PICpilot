/* 
 * File:   main.c
 *
 * Created on February 9, 2010, 10:53 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p33Fxxxx.h>
#include "OutputCompare.h"
#include "InputCapture.h"


short MSEC = 3125;
short UPPER_PWM = 6250;
short LOWER_PWM = 3125;
short MIDDLE_PWM = 4688;

/*
 * 
 */

void init_oc1(void)
{
// Initialize Output Compare Module
OC1CONbits.OCM = 0b000; // Disable Output Compare Module
OC1R = MIDDLE_PWM; // Write the duty cycle for the first PWM pulse = 1.5ms/4688
OC1RS = MIDDLE_PWM; // Write the duty cycle for the second PWM pulse] = 1.5ms/4688
OC1CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC1CONbits.OCM = 0b110; // Select the Output Compare mode (without fault protection)
}

void init_oc2(void)
{
// Initialize Output Compare Module
OC2CONbits.OCM = 0b000; // Disable Output Compare Module
OC2R = MIDDLE_PWM; // Write the duty cycle for the first PWM pulse = 1.5ms/4688
OC2RS = MIDDLE_PWM; // Write the duty cycle for the second PWM pulse] = 1.5ms/4688
OC2CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC2CONbits.OCM = 0b110; // Select the Output Compare mode (without fault protection)
}
void init_oc3(void)
{
// Initialize Output Compare Module
OC3CONbits.OCM = 0b000; // Disable Output Compare Module
OC3R = MIDDLE_PWM; // Write the duty cycle for the first PWM pulse = 1.5ms/4688
OC3RS = MIDDLE_PWM; // Write the duty cycle for the second PWM pulse] = 1.5ms/4688
OC3CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC3CONbits.OCM = 0b110; // Select the Output Compare mode (without fault protection)
}

void init_oc4(void)
{
// Initialize Output Compare Module
OC4CONbits.OCM = 0b000; // Disable Output Compare Module
OC4R = MIDDLE_PWM; // Write the duty cycle for the first PWM pulse = 1.5ms/4688
OC4RS = MIDDLE_PWM; // Write the duty cycle for the second PWM pulse] = 1.5ms/4688
OC4CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC4CONbits.OCM = 0b110; // Select the Output Compare mode (without fault protection)
}

void init_oc5(void)
{
//Initialize Output Compare Module
OC5CONbits.OCM = 0b000; // Disable Output Compare Module
OC5R = MIDDLE_PWM; // Write the duty cycle for the first PWM pulse = 1.5ms/4688
OC5RS = MIDDLE_PWM; // Write the duty cycle for the second PWM pulse] = 1.5ms/4688
OC5CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC5CONbits.OCM = 0b110; // Select the Output Compare mode (without fault protection)
}

void init_oc6(void)
{
// Initialize Output Compare Module
OC6CONbits.OCM = 0b000; // Disable Output Compare Module
OC6R = MIDDLE_PWM; // Write the duty cycle for the first PWM pulse = 1.5ms/4688
OC6RS = MIDDLE_PWM; // Write the duty cycle for the second PWM pulse] = 1.5ms/4688
OC6CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC6CONbits.OCM = 0b110; // Select the Output Compare mode (without fault protection)
}

void init_oc7(void)
{
// Initialize Output Compare Module
OC7CONbits.OCM = 0b000; // Disable Output Compare Module
OC7R = MIDDLE_PWM; // Write the duty cycle for the first PWM pulse = 1.5ms/4688
OC7RS = MIDDLE_PWM; // Write the duty cycle for the second PWM pulse] = 1.5ms/4688
OC7CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC7CONbits.OCM = 0b110; // Select the Output Compare mode (without fault protection)
}

void init_oc8(void)
{
// Initialize Output Compare Module
OC8CONbits.OCM = 0b000; // Disable Output Compare Module
OC8R = MIDDLE_PWM; // Write the duty cycle for the first PWM pulse = 1.5ms/4688
OC8RS = MIDDLE_PWM; // Write the duty cycle for the second PWM pulse] = 1.5ms/4688
OC8CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC8CONbits.OCM = 0b110; // Select the Output Compare mode (without fault protection)
}

void setPeriod(double time) ///////////////////FIX
{
//    Time is input in milliseconds
//    Using Timer 2:
     int pScale = T2CONbits.TCKPS;

    //If the prescaler is too small, it is incremented, and the corresponding
    //period and PWM are altered.
    if (time * MSEC > 0xFFFF)
    {
        pScale++;
        T2CONbits.TCKPS = pScale;

        MSEC /= 8;  //Divide the cycle/sec ratio by the prescaler

        //Reinitialize all the variables
        UPPER_PWM = MSEC * 2;
        MIDDLE_PWM = MSEC * 1.5;
        LOWER_PWM = MSEC;
        init_oc();

    }


        //PR2 = (unsigned int)(10 * MSEC);
     PR2 = (unsigned int)(time * MSEC);

}

void setPWM(int ocPin, int time)
{
    //Sets the PWM for subsequent pulses
    if (ocPin == 1)
    {
        OC1RS = time;
    }
    else if (ocPin == 2)
    {
        OC2RS = time;
    }
    else if (ocPin == 3)
    {
        OC3RS = time;
    }
    else if (ocPin == 4)
    {
        OC4RS = time;
    }
    else if (ocPin == 5)
    {
        OC5RS = time;
    }
    else if (ocPin == 6)
    {
        OC6RS = time;
    }
    else if (ocPin == 7)
    {
        OC7RS = time;
    }
    else if (ocPin == 8)
    {
        OC8RS = time;
    }
}

void init_oc(void)
{
    //Initialize each of the 8 OCs
    init_oc1();
    init_oc2();
    init_oc3();
    init_oc4();
    init_oc5();
    init_oc6();
    init_oc7();
    init_oc8();
}

void initOC(void){ //int argc, char** argv) {
    init_oc();
    //Initialize timer2
    init_t2();

}
