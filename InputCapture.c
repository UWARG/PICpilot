/*
 * File:   main.c
 *
 * Created on February 9, 2010, 10:53 AM
 */

// include files
#include <stdio.h>
#include <stdlib.h>
#include <p33Fxxxx.h>
#include "InputCapture.h"
#include "OutputCompare.h"


//Global variables: each variable is stored in an array to hold all the input
//                  channel values (from IC1 - IC8)


int t1[9];            // time one and time two capture for
int t2[9];            // each input channel

short checkic[9];     // flag bit
int icTimeDiff[9];

int Batt = 0;
int Throttle = 0;     // easy use variable names for icTime Differences
int Roll  = 0;
int Pitch = 0;
int Yaw = 0;
int Gear = 0;
int Aux1 = 0;



// time calculation stored in an array
void init_t2(void)    // Initialize and enable Timer2
{
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b01; // Select 1:8 Prescaler
    TMR2 = 0x00; // Clear timer register
    setPeriod(22.5);
    //PR2 = MSEC * 10; // Load the period value = 10ms
    IPC1bits.T2IP = 0x00; // Set Timer 2 Interrupt Priority Level
    IFS0bits.T2IF = 0; // Clear Timer 2 Interrupt Flag
    IEC0bits.T2IE = 0; // Disable Timer 2 interrupt
    T2CONbits.TON = 1; // Start Timer
}



//Input Capture #1 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC1Interrupt(void)
{
    if (PORTDbits.RD8 == 1)    // if IC signal is goes 0 --> 1, set t1 equal to
        t1[1]=IC1BUF;          // buffer value or else if signal goes 1 --> 0
    else                       // set t2 to buffer
    {
        t2[1]=IC1BUF;
        checkic[1] = 1;
    }
    IFS0bits.IC1IF=0;
}

//Input Capture #2 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC2Interrupt(void)
{
    if (PORTDbits.RD9 == 1)
        t1[2]=IC2BUF;
    else
    {
        t2[2]=IC1BUF;
        checkic[2] = 1;
    }
    IFS0bits.IC2IF=0;
}

//Input Capture #3 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC3Interrupt(void)
{
    if (PORTDbits.RD10 == 1)
        t1[3]=IC3BUF;
    else
    {
        t2[3]=IC1BUF;
        checkic[3] = 1;
    }
    IFS2bits.IC3IF=0;
}

//Input Capture #4 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC4Interrupt(void)
{
    if (PORTDbits.RD11 == 1)
        t1[4]=IC4BUF;
    else
    {
        t2[4]=IC1BUF;
        checkic[4] = 1;
    }
    IFS2bits.IC4IF=0;
}

//Input Capture #5 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC5Interrupt(void)
{
    if (PORTDbits.RD12 == 1)
        t1[5]=IC5BUF;
    else
    {
        t2[5]=IC1BUF;
        checkic[5] = 1;
    }
    IFS2bits.IC5IF=0;
}

//Input Capture #6 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC6Interrupt(void)
{
    if (PORTDbits.RD13 == 1)
        t1[6]=IC6BUF;
    else
    {
        t2[6]=IC1BUF;
        checkic[6] = 1;
    }
    IFS2bits.IC6IF=0;
}

//Input Capture #7 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC7Interrupt(void)
{
    if (PORTDbits.RD14 == 1)
        t1[7]=IC7BUF;
    else
    {
        t2[7]=IC1BUF;
        checkic[7] = 1;
    }
    IFS1bits.IC7IF=0;
}

//Input Capture #8 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC8Interrupt(void)
{
    if (PORTDbits.RD15 == 1)
        t1[8]=IC8BUF;
    else
    {
        t2[8]=IC1BUF;
        checkic[8] = 1;
    }
    IFS1bits.IC8IF=0;
}


void initInputCapture()     // Capture Interrupt Service Routine
{                           //    unsigned int timePeriod= 0;
    IC1CONbits.ICM = 0b00; // Disable Input Capture 1 module
    IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC1CONbits.ICI = 0b00; // Interrupt on every capture event
    IC1CONbits.ICM = 0b001; // Generate capture event on every Rising and Falling edge

    IC2CONbits.ICM = 0b00; // Disable Input Capture 2 module
    IC2CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC2CONbits.ICI = 0b00; // Interrupt on every capture event
    IC2CONbits.ICM = 0b001; // Generate capture event on every Rising edge

    IC3CONbits.ICM = 0b00; // Disable Input Capture 3 module
    IC3CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC3CONbits.ICI = 0b00; // Interrupt on every capture event
    IC3CONbits.ICM = 0b001; // Generate capture event on every Rising edge

    IC4CONbits.ICM = 0b00; // Disable Input Capture 4 module
    IC4CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC4CONbits.ICI = 0b00; // Interrupt on every capture event
    IC4CONbits.ICM = 0b001; // Generate capture event on every Rising edge

    IC4CONbits.ICM = 0b00; // Disable Input Capture 5 module
    IC4CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC4CONbits.ICI = 0b00; // Interrupt on every capture event
    IC4CONbits.ICM = 0b001; // Generate capture event on every Rising edge

    IC5CONbits.ICM = 0b00; // Disable Input Capture 6 module
    IC5CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC5CONbits.ICI = 0b00; // Interrupt on every capture event
    IC5CONbits.ICM = 0b001; // Generate capture event on every Rising edge

    IC6CONbits.ICM = 0b00; // Disable Input Capture 7 module
    IC6CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC6CONbits.ICI = 0b00; // Interrupt on every capture event
    IC6CONbits.ICM = 0b001; // Generate capture event on every Rising edge

    IC7CONbits.ICM = 0b00; // Disable Input Capture 8 module
    IC7CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC7CONbits.ICI = 0b00; // Interrupt on every capture event
    IC7CONbits.ICM = 0b001; // Generate capture event on every Rising edge

    IC8CONbits.ICM = 0b00; // Disable Input Capture 9 module
    IC8CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC8CONbits.ICI = 0b00; // Interrupt on every capture event
    IC8CONbits.ICM = 0b001; // Generate capture event on every Rising edge

    // Enable Capture Interrupt And Timer2
    IPC0bits.IC1IP = 1; // Setup IC1 interrupt priority level
    IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
    IEC0bits.IC1IE = 1; // Enable IC1 interrupt
}


// easy use variable names for icTime Differences
//void init_EasyVarNames()
//{
//    Batt = icTimeDiff[1];
//    Throttle = icTimeDiff[2];
//    Roll  = icTimeDiff[3];        //ROLL
//    Pitch = icTimeDiff[4];        //PITCH
//    Yaw = icTimeDiff[5];          //YAW
//    Gear = icTimeDiff[6];
//    Aux1 = icTimeDiff[7];
//}

 int* getICValues(){
            //Calculate and Update the Input Values

        short ic;
        for (ic = 1; ic <= 8; ic++)
        {
            //If knew data has been received on the given channel
            if (checkic[ic] == 1)
            {
              //If the second time is greater than the first time then we have not met roll over
              if(t2[ic] > t1[ic])
              {
                 icTimeDiff[ic] = (t2[ic] - t1[ic]);
                 checkic[ic] = 0;
              }
              //Roll over has been made, therefore do math to find the time difference
              //(Max time - t1) is the upper difference
              //Adding the second time (time from 0) gives you the total time difference
              else
              {
                 icTimeDiff[ic] = ((PR2 - t1[ic]) + t2[ic]);
                 checkic[ic] = 0;
              }
            }
        }
        return icTimeDiff;
}

// initialize function
void initIC()
{
    init_t2();
    initInputCapture();
}
