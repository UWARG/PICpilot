/*******************************************************************************
*
* FileName:        AutoRelay_v0_01.c
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       PIC24Fxxxx
*
*******************************************************************************/



/******************************* Header Files *********************************/
#include "p24F16KA101.h"


/*// FBS
#pragma config BWRP = OFF               // Table Write Protect Boot (Boot segment may be written)
#pragma config BSS = OFF                // Boot segment Protect (No boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Code Flash Write Protection bit (General segment may be written)
#pragma config GCP = OFF                // General Segment Code Flash Code Protection bit (No protection)

// FOSCSEL
#pragma config FNOSC = FRCDIV           // Oscillator Select (8 MHz FRC oscillator with divide-by-N (FRCDIV))
#pragma config IESO = ON                // Internal External Switch Over bit (Internal External Switchover mode enabled (Two-Speed Start-up enabled))

// FOSC
#pragma config POSCMOD = NONE           // Primary Oscillator Configuration bits (Primary oscillator disabled)
#pragma config OSCIOFNC = OFF           // CLKO Enable Configuration bit (CLKO output signal is active on the OSCO pin)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8 MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary oscillator configured for high-power operation)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPS = PS4096           // Watchdog Timer Postscale Select bits (1:4,096)
#pragma config FWPSA = PR128            // WDT Prescaler (WDT prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected; windowed WDT disabled)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (WDT enabled)

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default location for SCL1/SDA1 pins)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
#pragma config MCLRE = ON               // MCLR Pin Enable bit (MCLR pin enabled; RA5 input pin disabled)

// FICD
#pragma config ICS = PGx1               // ICD Pin Placement Select bits (PGC1/PGD1 are used for programming and debugging the device)

// FDS
#pragma config DSWDTPS = DSWDTPSF       // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses LPRC as reference clock)
#pragma config RTCOSC = SOSC            // RTCC Reference Clock Select bit (RTCC uses SOSC as reference clock)
#pragma config DSBOREN = ON             // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)

*/




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
unsigned int timePeriod1 = 0;         // Used in interrupt
unsigned int timePeriod2 = 0;
unsigned int t[3] = {0,0,0};                // Used in interrupt
unsigned int i = 0;
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
void __attribute__((__interrupt__,no_auto_psv)) _IC1Interrupt(void)
{
    t[i] = IC1BUF;
    if (t[i] > t[(i+2)%3])
    {
        timePeriod1 = t[i] - t[(i+2)%3];
    }
    else{
        timePeriod1 = (PR2 - t[(i+2)%3]) + t[i];
    }
    if (t[(i+2)%3] > t[(i+1)%3]) {
        timePeriod2 = t[(i+2)%3] - t[(i+1)%3];
    }
    else {
        timePeriod2 = (PR2 - t[(i+1)%3]) + t[(i+2)%3];
    }

    if (timePeriod1 > timePeriod2)
	ch8_position = timePeriod2;
    else
        ch8_position = timePeriod1;
        
        //Setup for next interrupt
        i++;
        i = i%3;

        //clear watchdog
        asm("CLRWDT");
	IFS0bits.IC1IF=0;
}

/********************************* Init() *************************************/
void Init(void)
{
    //TODO:Set IC pin as input
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
//void readI2C(unsigned char sensor)
//{
//	char byteToRead = 0x07;
//	char status;
//	unsigned char *pRead;
//	unsigned char rx_data[2];	// "MCHP I2C"
//	pRead = rx_data;
//
//		while (1)
//		{
//			StartI2C1();
//			IdleI2C1();
//			Delay_ms(1);
//			MasterWriteI2C1(sensor); //transmit write command
//			IdleI2C1();
//			if( I2C1STATbits.ACKSTAT ) //end message if transmission fails
//			{
//				StopI2C1();
//				IdleI2C1();
//				break;
//			}
//
//			MasterWriteI2C1(byteToRead); //transmit which byte to read
//			IdleI2C1();
//			if( I2C1STATbits.ACKSTAT ) //end message if transmission fails
//			{
//				StopI2C1();
//				IdleI2C1();
//				break;
//			}
//			StopI2C1();
//			IdleI2C1();
//
//			StartI2C1();
//			IdleI2C1();
//
//			MasterWriteI2C1(sensor + 1); //transmit read command
//			IdleI2C1();
//			if( I2C1STATbits.ACKSTAT ) 	//end message if transmission fails
//			{
//				StopI2C1();
//				IdleI2C1();
//				break;
//			}
//
//			status = MastergetsI2C1(2, pRead, 900);		//read byte
//			//if (status==0){PORTAbits.RA6 = 1;}
//			StopI2C1();					//Send the Stop condition
//			IdleI2C1();					//Wait to complete
//			break;
//
//		}
//		dataI2C_1 = pRead[0];
//		dataI2C_2 = pRead[1];
//}

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

                //The window on the controller is 43%-50%
		if ((ch8_position > 500) && (ch8_position < 1500)) //Controller Setting is: +47%, -33% //(ch8_position > 450)
		{
			PORTBbits.RB7 = 1;
		}
		else
		{
			PORTBbits.RB7 = 0;
		}
	}
}
