#include <p33FJ256GP710.h>
#include "UART2.h"

#if PATH_MANAGER && !GPS_OLD
UART_RX_Buffer _buff;
#endif

void InitUART2()
{
	// This is an EXAMPLE, so brutal typing goes into explaining all bit sets

	// The HPC16 board has a DB9 connector wired to UART2, so we will
	// be configuring this port only

	// configure U2MODE
	U2MODEbits.UARTEN = 0;	// Bit15 TX, RX DISABLED, ENABLE at end of func
	//U2MODEbits.notimplemented;	// Bit14
	U2MODEbits.USIDL = 0;	// Bit13 Continue in Idle
	U2MODEbits.IREN = 0;	// Bit12 No IR translation
	U2MODEbits.RTSMD = 0;	// Bit11 Simplex Mode
	//U2MODEbits.notimplemented;	// Bit10
	U2MODEbits.UEN = 0;		// Bits8,9 TX,RX enabled, CTS,RTS not
	U2MODEbits.WAKE = 0;	// Bit7 No Wake up (since we don't sleep here)
	U2MODEbits.LPBACK = 0;	// Bit6 No Loop Back
	U2MODEbits.ABAUD = 0;	// Bit5 No Autobaud (would require sending '55')
	U2MODEbits.URXINV = 0;	// Bit4 IdleState = 1  (for dsPIC)
	U2MODEbits.BRGH = 0;	// Bit3 16 clocks per bit period
	U2MODEbits.PDSEL = 0b00;	// Bits1,2 8bit, No Parity
	U2MODEbits.STSEL = 0;	// Bit0 One Stop Bit

        //***NOTE That Mitch Changed this code and the Baud Rate can be automatically calculated at the top
	// Load a value into Baud Rate Generator.  Example is for 9600.
	// See section 19.3.1 of datasheet.
	//  U1BRG = (Fcy/(16*BaudRate))-1
	//  U1BRG = (3.7M/(16*9600))-1
	//  U1BRG = 23

        //UART Baud Rate Calculated Values...
        //9600 = 23
        //19200 = 11
        //38400 = 5
        //115200 = 1
        U2BRG = 5;//Baud_Rate;	// 40Mhz osc, ___ Baud

	// Load all values in for U1STA SFR
	U2STAbits.UTXISEL1 = 0;	//Bit15 Int when Char is transferred (1/2 config!)
	U2STAbits.UTXINV = 0;	//Bit14 N/A, IRDA config
	U2STAbits.UTXISEL0 = 0;	//Bit13 Other half of Bit15
	//U2STAbits.notimplemented = 0;	//Bit12
	U2STAbits.UTXBRK = 0;	//Bit11 Disabled
	U2STAbits.UTXEN = 0;	//Bit10 TX pins controlled by periph
	U2STAbits.UTXBF = 0;	//Bit9 *Read Only Bit*
	U2STAbits.TRMT = 0;	//Bit8 *Read Only bit*
	U2STAbits.URXISEL = 0;	//Bits6,7 Int. on character recieved
	U2STAbits.ADDEN = 0;	//Bit5 Address Detect Disabled
	U2STAbits.RIDLE = 0;	//Bit4 *Read Only Bit*
	U2STAbits.PERR = 0;		//Bit3 *Read Only Bit*
	U2STAbits.FERR = 0;		//Bit2 *Read Only Bit*
	U2STAbits.OERR = 0;		//Bit1 *Read Only Bit*
	U2STAbits.URXDA = 0;	//Bit0 *Read Only Bit*

//	IPC7 = 0x4400;	// Mid Range Interrupt Priority level, no urgent reason

        IPC7 = 0x4400;	// Mid Range Interrupt Priority level, no urgent reason

	IFS1bits.U2TXIF = 0;	// Clear the Transmit Interrupt Flag
	IEC1bits.U2TXIE = 0;	// Enable Transmit Interrupts
	IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag

#if PATH_MANAGER && !GPS_OLD
	IEC1bits.U2RXIE = 1;	// Enable Recieve Interrupts
#else
        IEC1bits.U2RXIE = 0;	// Enable Recieve Interrupts
#endif

#if PATH_MANAGER && !GPS_OLD
	U1MODEbits.UARTEN = 1;	// And turn the peripheral on
	U1STAbits.UTXEN = 1;
#endif
	U2MODEbits.UARTEN = 1;	// And turn the peripheral on

	U2STAbits.UTXEN = 1;

        //PORTBbits.RB14 = 0;
	// I think I have the thing working now.


        /* wait at least 104 usec (1/9600) before sending first char */
        int i = 0;
        while (i < 4160)
        {
            Nop();
            i++;
        }
}

void UART2_SendChar(char data)
{
    U2TXREG = data;
    while(U2STAbits.TRMT == 0);
#if !PATH_MANAGER
    U2STAbits.TRMT = 0;
#endif
}

void UART2_SendString(char *s)
{
    do
    {
        UART2_SendChar(*s);
        s++;
        //Delay10TCYx(250);
    }while(*s != 0);

          UART2_SendChar('\n');
}

#if PATH_MANAGER && !GPS_OLD
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt( void )
{
    write_rx_buffer(U2RXREG,&_buff);
    IFS1bits.U2RXIF = 0;
}

void init_rx_buffer(UART_RX_Buffer *buff)
{
    buff->get = 0;
    buff->put = 1;
}
void write_rx_buffer(char c, UART_RX_Buffer *buff)
{
    buff->data[buff->put] = c;
    if(buff->put < 255)
        buff->put++;
    else
        buff->put = 0;

    if(buff->put == buff->get)
    {
        buff->get++;
        if(buff->get > 255)
            buff->get = 0;
    }
}
char read_rx_buffer(UART_RX_Buffer *buff)
{
    char c = buff->data[buff->get];
    short oldGet = buff->get;
    if(buff->get < 255)
        buff->get++;
    else
        buff->get = 0;

    if(buff->get == buff->put)
    {
        buff->get = oldGet;
        return 0;
    }

    return c;
}
#endif