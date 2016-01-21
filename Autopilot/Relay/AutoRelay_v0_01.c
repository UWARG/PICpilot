/*******************************************************************************
*
* FileName:        AutoRelay_v0_01.c
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       PIC24Fxxxx
*
*******************************************************************************/

/******************************* Header Files *********************************/
#include "p24F16KA101.h"
#include <i2c.h>

/********************************* Defines ************************************/
#define USE_AND_OR	                // To enable AND_OR mask setting for I2C.
#define Fosc	    (16000000) 	    // crystal
#define Fcy		    (Fosc*4/2)	    // w.PLL (Instruction Per Second)
#define Fsck	    100000		    // 400kHz I2C
#define I2C_BRG	    ((Fcy/2/Fsck)-1)

/************************* Function Prototypes ********************************/
void Init(void);
void Delay_ms(unsigned int millisec);

void readI2C(unsigned char sensor);

/**************************** Global Variables ********************************/
unsigned int timePeriod= 0;         // Used in interrupt
unsigned int t1 = 0;                // Used in interrupt
unsigned int t2 = 0;                // Used in interrupt
unsigned int ch8_position = 0;      // Used in Main()


unsigned char data = (char)0x00;
unsigned char gga[65];
unsigned char vtg[65];
unsigned char sp[100];
unsigned char apiString[100];
unsigned int spIndex;
unsigned char checkSum;
unsigned int comma;
unsigned int dataValid;
unsigned char dataI2C_1;
unsigned char dataI2C_2;

/******************************* Interrupt ************************************/
void __attribute__((__interrupt__)) _IC1Interrupt(void)
{
	if (PORTAbits.RA6 == 1)
	{
		t1=IC1BUF;
	}
	else if (PORTAbits.RA6 == 0)
	{
		t2=IC1BUF;
		if(t2>t1)
		{
			timePeriod = t2-t1;
		}
		else
		{
			timePeriod = (PR2 - t1) + t2;
		}
	}
	ch8_position = timePeriod;

	//clear watchdog
	asm("CLRWDT");

	IFS0bits.IC1IF=0;
}

/********************************* Init() *************************************/
void Init(void)
{
	PORTBbits.RB7 = 0;
	TRISBbits.TRISB7 = 0;       // Set RB7 as an output
	AD1PCFG = 0xFFFF;			// Use inputs in digital mode
	IC1CONbits.ICM = 0b00;      // Disable Input Capture 7 module
    IC1CONbits.ICTMR = 1;       // Select Timer2 as the IC7 Time base
    IC1CONbits.ICM = 0b001;     // Generate capture event on every clock event

    IPC0bits.IC1IP = 5;         // Setup IC7 interrupt priority level
    IFS0bits.IC1IF = 0;         // Clear IC7 Interrupt Status Flag
    IEC0bits.IC1IE = 1;         // Enable IC7 interrupt

    T2CONbits.TON = 0;          // Disable Timer
    T2CONbits.TCS = 0;          // Select internal instruction cycle clock
    T2CONbits.TGATE = 0;        // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b01;     // Select 1:1 Prescaler

    PR2 = 65000;                // Load the period value
    TMR2 = 0x00;                // Clear timer register
    IFS0bits.T2IF = 0;          // Clear Timer 2 Interrupt Flag

    T2CONbits.TON = 1;          // Start Timer

/*
    ODCAbits.ODA6 = 0;			// for diagnostic/debuging LED and input
    IPC1bits.T2IP = 0x01;         // Set Timer 2 Interrupt Priority Level
    IEC0bits.T2IE = 1;            // Enable Timer 2 interrupt
    PORTAbits.RA6 = 1;
    U2STA = 0;					//setup uart 2 - output
    U2MODE = 0x8000;			//8 bit, one stop, no parity
    U2BRG = 8;					//set baud to 115200
    U1STA = 0;					//setup uart 1 - gps
    U1MODE = 0x8000;			//8 bit, one stop, no parity
    U1BRG = 8;					//set baud to 115200
    U2MODEbits.UARTEN = 1;		//enable uarts and tx ouputs
    U2STAbits.UTXEN = 1;
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
    OpenI2C1( I2C_ON, I2C_BRG );//open i2c channel
*/
}

/****************************** Delay_ms() ************************************/
void Delay_ms(unsigned int millisec){
	unsigned int i;
	unsigned int j;

	for (i=0; i<=millisec; i++)
	{
		for (j=0; j<=687; j++)
		{
			asm("nop");
		}
	}
}

/******************************* readI2C() ************************************/
void readI2C(unsigned char sensor)
{
	char byteToRead = 0x07;
	char status;
	unsigned char *pRead;
	unsigned char rx_data[2];	// "MCHP I2C"
	pRead = rx_data;

		while (1)
		{
			StartI2C1();
			IdleI2C1();
			Delay_ms(1);
			MasterWriteI2C1(sensor); //transmit write command
			IdleI2C1();
			if( I2C1STATbits.ACKSTAT ) //end message if transmission fails
			{
				StopI2C1();
				IdleI2C1();
				break;
			}

			MasterWriteI2C1(byteToRead); //transmit which byte to read
			IdleI2C1();
			if( I2C1STATbits.ACKSTAT ) //end message if transmission fails
			{
				StopI2C1();
				IdleI2C1();
				break;
			}
			StopI2C1();
			IdleI2C1();

			StartI2C1();
			IdleI2C1();

			MasterWriteI2C1(sensor + 1); //transmit read command
			IdleI2C1();
			if( I2C1STATbits.ACKSTAT ) 	//end message if transmission fails
			{
				StopI2C1();
				IdleI2C1();
				break;
			}

			status = MastergetsI2C1(2, pRead, 900);		//read byte
			//if (status==0){PORTAbits.RA6 = 1;}
			StopI2C1();					//Send the Stop condition
			IdleI2C1();					//Wait to complete
			break;

		}
		dataI2C_1 = pRead[0];
		dataI2C_2 = pRead[1];
}

/********************************* Main() *************************************/
// range of ch8_position relative to remote controller:
// 575 ms all the way down
// 1000 ms all the way up
// setting on remote-controller:
// 47 off
// 48-52 on
// 53 off

int main (void)
{
    Init();

	while(1)
	{
           
		if (ch8_position > 450)//(((ch8_position > 400) && (ch8_position < 425))||((ch8_position > 530) && (ch8_position < 555)))//(((ch8_position > 565) && (ch8_position < 590))||((ch8_position > 930) && (ch8_position < 955)))
		{
			PORTBbits.RB7 = 1;
		}
		else
		{
			PORTBbits.RB7 = 0;
		}
	}
}
