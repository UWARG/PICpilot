/**
 * @file UART.c
 * @author Serge Babayan
 * @date February 14, 2017
 */

#include "UART.h"
#include <p33FJ256GP710A.h>
#include "../../Common/clock.h"

/**
 * The chip requires the baud rate register to be set to a value that corresponds
 * to the processor frequency. The formula can be found on the data sheet. This
 * takes an ordinary baud rate value in and returns that
 */
#define BRGVAL(bd) (((CLOCK_FREQUENCY/bd)/16)-1)

#define INTERFACE_DISABLED 0
#define INTERFACE_ENABLED 1
#define INTERFACE_INITIALIZED 2

static char uart1_status = INTERFACE_UART1_ENABLED;
static char uart2_status = INTERFACE_UART2_ENABLED;

void initUART(unsigned char interface, unsigned long int baudrate)
{

    //we don't initialize twice or don't initialize if the interface is disabled
    if (interface == 1 && uart1_status == INTERFACE_ENABLED) {
        return;
    } else if (interface == 2 && uart2_status == INTERFACE_ENABLED) {
        U2MODEbits.UARTEN = 0; //Disable UART2 while we're configuring it
        U2MODEbits.USIDL = 0; //Keep interface on in idle mode
        U2MODEbits.IREN = 0; //No IR translation
        U2MODEbits.RTSMD = 0; //Simplex Mode for RTS pin
        U2MODEbits.WAKE = 0; //No wakeup (we dont sleep)
        U2MODEbits.LPBACK = 0; //No Loop Back
        U2MODEbits.ABAUD = 0; //No Auto baudrate detection (would require sending '55')
        
        //important settings:
        U2MODEbits.UEN = 0; //TX,RX enabled, CTS,RTS not (no flow control)
        U2MODEbits.URXINV = 0; //IdleState = 1  (for dsPIC)
        U2MODEbits.BRGH = 0; // 16 clocks per bit period (Standard Speed Mode)
        U2MODEbits.PDSEL = 0b00; // Bits1,2 8bit, No Parity
        U2MODEbits.STSEL = 0; // Bit0 One Stop Bit
        
        //set the baud rate
        U2BRG = BRGVAL(baudrate);

        // Load all values in for U1STA SFR
        U2STAbits.UTXISEL1 = 0; //Bit15 Int when Char is transferred (1/2 config!)
        U2STAbits.UTXINV = 0; //Bit14 N/A, IRDA config
        U2STAbits.UTXISEL0 = 0; //Bit13 Other half of Bit15
        //U2STAbits.notimplemented = 0;	//Bit12
        U2STAbits.UTXBRK = 0; //Bit11 Disabled
        U2STAbits.UTXEN = 0; //Bit10 TX pins controlled by periph
        U2STAbits.UTXBF = 0; //Bit9 *Read Only Bit*
        U2STAbits.TRMT = 0; //Bit8 *Read Only bit*
        U2STAbits.URXISEL = 0; //Bits6,7 Int. on character recieved
        U2STAbits.ADDEN = 0; //Bit5 Address Detect Disabled
        U2STAbits.RIDLE = 0; //Bit4 *Read Only Bit*
        U2STAbits.PERR = 0; //Bit3 *Read Only Bit*
        U2STAbits.FERR = 0; //Bit2 *Read Only Bit*
        U2STAbits.OERR = 0; //Bit1 *Read Only Bit*
        U2STAbits.URXDA = 0; //Bit0 *Read Only Bit*

        //	IPC7 = 0x4400;	// Mid Range Interrupt Priority level, no urgent reason

        IPC7 = 0x4400; // Mid Range Interrupt Priority level, no urgent reason

        IFS1bits.U2TXIF = 0; // Clear the Transmit Interrupt Flag
        IEC1bits.U2TXIE = 0; // Enable Transmit Interrupts
        IFS1bits.U2RXIF = 0; // Clear the Recieve Interrupt Flag

        IEC1bits.U2RXIE = 0; // Enable Recieve Interrupts

        U2MODEbits.UARTEN = 1; // And turn the peripheral on

        U2STAbits.UTXEN = 1;

        //PORTBbits.RB14 = 0;
        // I think I have the thing working now.


        /* wait at least 104 usec (1/9600) before sending first char */
        int i = 0;
        while (i < 4160) {
            Nop();
            i++;
        }
    }
}