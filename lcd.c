/**********************************************************************
* © 2006 Microchip Technology Inc.
*
* FileName:        lcd.c
* Dependencies:    p33FJ256GP710.h
*                  lcd.h
*                  delay.h
* Processor:       dsPIC33F
* Compiler:        MPLAB® C30 v2.01 or higher
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Inc. (“Microchip”) licenses this software to you
* solely for use with Microchip dsPIC® digital signal controller
* products. The software is owned by Microchip and is protected under
* applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED “AS IS.”  MICROCHIP EXPRESSLY DISCLAIMS ANY
* WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
* BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
* DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
* PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
* BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
* ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author            Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Richard Fischer   07/14/05  Explorer 16 board LCD support
* Priyabrata Sinha  01/27/06  Ported to non-prototype devices
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:
*
**********************************************************************/

#include "p33FJ256GP710A.h"
#include "lcd.h"
#include "delay.h"
	
		
/* 
   For Explorer 16 board, here are the data and control signal definitions
   RS -> RB15
   E  -> RD4
   RW -> RD5
   DATA -> RE0 - RE7   
*/

// Control signal data pins 
#define  RW  LATDbits.LATD5       // LCD R/W signal
#define  RS  LATBbits.LATB15      // LCD RS signal
#define  E   LATDbits.LATD4       // LCD E signal 
//#define  E   LATFbits.LATF1       // LCD E signal

// Control signal pin direction 
#define  RW_TRIS	TRISDbits.TRISD5 
#define  RS_TRIS	TRISBbits.TRISB15
#define  E_TRIS		TRISDbits.TRISD4
//#define  E_TRIS		TRISFbits.TRISF1

// Data signals and pin direction
#define  DATA      LATE           // Port for LCD data
#define  DATAPORT  PORTE
#define  TRISDATA  TRISE          // I/O setup for data Port



/****************************************************************************/
/*****	LCD SUBROUTINE  *****/

void Init_LCD( void )             // initialize LCD display
{
	// 15mS delay after Vdd reaches nnVdc before proceeding with LCD initialization
	// not always required and is based on system Vdd rise rate
	Delay(Delay_15mS_Cnt);                  // 15ms delay
			
	/* set initial states for the data and control pins */
	LATE &= 0xFF00;	
    RW = 0;                       // R/W state set low
	RS = 0;                       // RS state set low
	E = 0;                        // E state set low

	/* set data and control pins to outputs */
	TRISE &= 0xFF00;
 	RW_TRIS = 0;                  // RW pin set as output
	RS_TRIS = 0;                  // RS pin set as output
	E_TRIS = 0;                   // E pin set as output

	/* 1st LCD initialization sequence */
	DATA &= 0xFF00;
    DATA |= 0x0038;
    E = 1;	
    Nop();
    Nop();
    Nop();
    E = 0;                        // toggle E signal
   	Delay(Delay_5mS_Cnt);         // 5ms delay
      
	/* 2nd LCD initialization sequence */
	DATA &= 0xFF00;
    DATA |= 0x0038;
    E = 1;	
    Nop();
    Nop();
    Nop();	
    E = 0;                        // toggle E signal
    Delay_Us( Delay200uS_count ); // 200uS delay

	/* 3rd LCD initialization sequence */
	DATA &= 0xFF00;
    DATA |= 0x0038;
    E = 1;		
    Nop();
    Nop();
    Nop();	
    E = 0;                        // toggle E signal
    Delay_Us( Delay200uS_count ); // 200uS delay

    lcd_cmd( 0x38 );              // function set
    lcd_cmd( 0x0C );              // Display on/off control, cursor blink off (0x0C)
    lcd_cmd( 0x06 );			  // entry mode set (0x06)
}


void lcd_cmd( char cmd )          // subroutiune for lcd commands
{
//	TRISD &= 0xFF00;              // ensure RD0 - RD7 are outputs
	DATA &= 0xFF00;               // prepare RD0 - RD7
    DATA |= cmd;                  // command byte to lcd
	RW = 0;                       // ensure RW is 0
    RS = 0;
    E = 1;                        // toggle E line
    Nop();
    Nop();
    Nop();
    E = 0;
   	Delay(Delay_5mS_Cnt);         // 5ms delay
}


void lcd_data( char data )        // subroutine for lcd data
{
//	TRISD &= 0xFF00;              // ensure RD0 - RD7 are outputs
	RW = 0;       				 // ensure RW is 0
    RS = 1;                       // assert register select to 1
	DATA &= 0xFF00;               // prepare RD0 - RD7
    DATA |= data;                 // data byte to lcd
    E = 1;				
 	Nop();
    Nop();
    Nop();
    E = 0;                       // toggle E signal
    RS = 0;                      // negate register select to 0
    Delay_Us( Delay200uS_count ); // 200uS delay
    Delay_Us( Delay200uS_count ); // 200uS delay
}


void SendToLCD( unsigned char *data, unsigned char count )
{
  	while ( count )
	{
		lcd_data( *data++ );
		count --;
	}	
}
