/*
 * File:   InputCapture.c
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


int t1[8];            // Time 1 and Time 2 capture for
int t2[8];            // each input channel

short icInterruptFlag[8];     // Flag bit
unsigned int icTime[8];       // Time between interrupts (time between each pulse)

// Time calculation stored in an array
void initTimer2(void)    // Initialize and enable Timer2
{
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b01; // Select 1:8 Prescaler
    TMR2 = 0x00; // Clear timer register
    setPeriod(20);
    //PR2 = MSEC * 20; // Load the period value = 20ms                        //
    IPC1bits.T2IP = 0x01; // Set Timer 2 Interrupt Priority Level - Lowest
    IFS0bits.T2IF = 0; // Clear Timer 2 Interrupt Flag
    IEC0bits.T2IE = 1; // Enable Timer 2 interrupt
    T2CONbits.TON = 1; // Start Timer
}



//Input Capture #1 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC1Interrupt(void)
{
    if (PORTDbits.RD8 == 1)    // if IC signal is goes 0 --> 1, set t1 equal to
        t1[0]=IC1BUF;          // buffer value or else if signal goes 1 --> 0
    else                       // set t2 to buffer
    {
        t2[0]=IC1BUF;
        icInterruptFlag[0] = 1;
    }
    IFS0bits.IC1IF=0;
}

//Input Capture #2 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC2Interrupt(void)
{
    if (PORTDbits.RD9 == 1)
        t1[1]=IC2BUF;
    else
    {
        t2[1]=IC2BUF;
        icInterruptFlag[1] = 1;
    }
    IFS0bits.IC2IF=0;
}

//Input Capture #3 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC3Interrupt(void)
{
    if (PORTDbits.RD10 == 1)
        t1[2]=IC3BUF;
    else
    {
        t2[2]=IC3BUF;
        icInterruptFlag[2] = 1;
    }
    IFS2bits.IC3IF=0;
}

//Input Capture #4 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC4Interrupt(void)
{
    if (PORTDbits.RD11 == 1)
        t1[3]=IC4BUF;
    else
    {
        t2[3]=IC4BUF;
        icInterruptFlag[3] = 1;
    }
    IFS2bits.IC4IF=0;
}

//Input Capture #5 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC5Interrupt(void)
{
    if (PORTDbits.RD12 == 1)
        t1[4]=IC5BUF;
    else
    {
        t2[4]=IC5BUF;
        icInterruptFlag[4] = 1;
    }
    IFS2bits.IC5IF=0;
}

//Input Capture #6 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC6Interrupt(void)
{
    if (PORTDbits.RD13 == 1)
        t1[5]=IC6BUF;
    else
    {
        t2[5]=IC6BUF;
        icInterruptFlag[5] = 1;
    }
    IFS2bits.IC6IF=0;
}

//Input Capture #7 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC7Interrupt(void)
{
    if (PORTDbits.RD14 == 1)
        t1[6]=IC7BUF;
    else
    {
        t2[6]=IC7BUF;
        icInterruptFlag[6] = 1;
    }
    IFS1bits.IC7IF=0;
}

//Input Capture #8 Interrupt Function
void __attribute__((__interrupt__,no_auto_psv)) _IC8Interrupt(void)
{
    if (PORTDbits.RD15 == 1)
        t1[7]=IC8BUF;
    else
    {
        t2[7]=IC8BUF;
        icInterruptFlag[7] = 1;
    }
    IFS1bits.IC8IF=0;
}


void initInputCapture(char initIC)     // Capture Interrupt Service Routine
{                           //    unsigned int timePeriod= 0;
    if (initIC & 0b01){
        IC1CONbits.ICM = 0b00; // Disable Input Capture 1 module
        IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
        IC1CONbits.ICI = 0b11; // Interrupt on every capture event
        IC1CONbits.ICM = 0b001; // Generate capture event on every Rising and Falling edge

        // Enable Capture Interrupt And Timer2
        IPC0bits.IC1IP = 7; // Setup IC1 interrupt priority level - Highest
        IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
        IEC0bits.IC1IE = 1; // Enable IC1 interrupt
    }
    if (initIC & 0b10){
        IC2CONbits.ICM = 0b00; // Disable Input Capture 2 module
        IC2CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
        IC2CONbits.ICI = 0b11; // Interrupt on every capture event
        IC2CONbits.ICM = 0b001; // Generate capture event on every Rising edge

        // Enable Capture Interrupt And Timer2
        IPC1bits.IC2IP = 7; // Setup IC2 interrupt priority level - Highest
        IFS0bits.IC2IF = 0; // Clear IC2 Interrupt Status Flag
        IEC0bits.IC2IE = 1; // Enable IC2 interrupt
    }
    if (initIC & 0b100){
        IC3CONbits.ICM = 0b00; // Disable Input Capture 3 module
        IC3CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
        IC3CONbits.ICI = 0b11; // Interrupt on every capture event
        IC3CONbits.ICM = 0b001; // Generate capture event on every Rising edge

        // Enable Capture Interrupt And Timer2
        IPC9bits.IC3IP = 7; // Setup IC3 interrupt priority level - Highest
        IFS2bits.IC3IF = 0; // Clear IC3 Interrupt Status Flag
        IEC2bits.IC3IE = 1; // Enable IC3 interrupt
    }
    if (initIC & 0b1000){
        IC4CONbits.ICM = 0b00; // Disable Input Capture 4 module
        IC4CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
        IC4CONbits.ICI = 0b11; // Interrupt on every capture event
        IC4CONbits.ICM = 0b001; // Generate capture event on every Rising edge

        // Enable Capture Interrupt And Timer2
        IPC9bits.IC4IP = 7; // Setup IC4 interrupt priority level - Highest
        IFS2bits.IC4IF = 0; // Clear IC4 Interrupt Status Flag
        IEC2bits.IC4IE = 1; // Enable IC4 interrupt
    }
    if (initIC & 0b10000){
        IC5CONbits.ICM = 0b00; // Disable Input Capture 5 module
        IC5CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
        IC5CONbits.ICI = 0b11; // Interrupt on every capture event
        IC5CONbits.ICM = 0b001; // Generate capture event on every Rising edge

        // Enable Capture Interrupt And Timer2
        IPC9bits.IC5IP = 7; // Setup IC5 interrupt priority level - Highest
        IFS2bits.IC5IF = 0; // Clear IC5 Interrupt Status Flag
        IEC2bits.IC5IE = 1; // Enable IC5 interrupt
    }
    if (initIC & 0b100000){
        IC6CONbits.ICM = 0b00; // Disable Input Capture 6 module
        IC6CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
        IC6CONbits.ICI = 0b11; // Interrupt on every capture event
        IC6CONbits.ICM = 0b001; // Generate capture event on every Rising edge

        // Enable Capture Interrupt And Timer2
        IPC10bits.IC6IP = 7; // Setup IC6 interrupt priority level - Highest
        IFS2bits.IC6IF = 0; // Clear IC6 Interrupt Status Flag
        IEC2bits.IC6IE = 1; // Enable IC6 interrupt
    }
    if (initIC & 0b1000000){
        IC7CONbits.ICM = 0b00; // Disable Input Capture 7 module
        IC7CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
        IC7CONbits.ICI = 0b11; // Interrupt on every capture event
        IC7CONbits.ICM = 0b001; // Generate capture event on every Rising edge

        // Enable Capture Interrupt And Timer2
        IPC5bits.IC7IP = 7; // Setup IC7 interrupt priority level - Highest
        IFS1bits.IC7IF = 0; // Clear IC7 Interrupt Status Flag
        IEC1bits.IC7IE = 1; // Enable IC7 interrupt
    }
    if (initIC & 0b10000000){
        IC8CONbits.ICM = 0b00; // Disable Input Capture 8 module
        IC8CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
        IC8CONbits.ICI = 0b11; // Interrupt on every capture event
        IC8CONbits.ICM = 0b001; // Generate capture event on every Rising edge

        // Enable Capture Interrupt And Timer2
        IPC5bits.IC8IP = 7; // Setup IC8 interrupt priority level - Highest
        IFS1bits.IC8IF = 0; // Clear IC8 Interrupt Status Flag
        IEC1bits.IC8IE = 1; // Enable IC8 interrupt
    }

}

// initialize function
void initIC(char initIC)
{
    initInputCapture(initIC);
    initTimer2();
}

unsigned int* getICValues(){
            //Calculate and Update the Input Values

        short ic;
        for (ic = 0; ic < 8; ic++)
        {
            //If new data has been received on the given channel
            if (icInterruptFlag[ic] == 1)
            {
              //If the second time is greater than the first time then we have not met roll over
              if(t2[ic] > t1[ic])
              {
                 icTime[ic] = (t2[ic] - t1[ic]);
                 icInterruptFlag[ic] = 0;
              }
              //Roll over has been made, therefore do math to find the time difference
              //(Max time - t1) is the upper difference
              //Adding the second time (time from 0) gives you the total time difference
              else
              {
                 icTime[ic] = ((PR2 - t1[ic]) + t2[ic]);
                 icInterruptFlag[ic] = 0;

              }
            }
        }
        return icTime;
}

unsigned int getICValue(unsigned char channel){
    unsigned int ic = (unsigned int)channel;
    //If new data has been received on the given channel
    if (icInterruptFlag[ic] == 1)
    {
      //If the second time is greater than the first time then we have not met roll over
      if(t2[ic] > t1[ic])
      {
         icTime[ic] = (t2[ic] - t1[ic]);
         icInterruptFlag[ic] = 0;
      }
      //Roll over has been made, therefore do math to find the time difference
      //(Max time - t1) is the upper difference
      //Adding the second time (time from 0) gives you the total time difference
      else
      {
         icTime[ic] = ((PR2 - t1[ic]) + t2[ic]);
         icInterruptFlag[ic] = 0;

      }
    }
    return icTime[ic];
}
