/**
 * @file UART.c
 * @author Serge Babayan
 * @date February 14, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "UART.h"
#include "p24F16KA101.h"
#include "../Common/Utilities/ByteQueue.h"
#include <stdint.h>

/**
 * The chip requires the baud rate register to be set to a value that corresponds
 * to the processor frequency. The formula can be found on the data sheet. This
 * takes an ordinary baud rate value in and returns that
 */
#define BRGVAL(bd) (((16000000/bd)/16) - 1)

void initUART(uint8_t interface, uint32_t baudrate)
{
    //we don't initialize twice or don't initialize if the interface is disabled
    if (interface == 1) {

        U1MODEbits.UARTEN = 0; //Disable UART while we're configuring it
        U1MODEbits.USIDL = 0; //Keep interface on in idle mode
        U1MODEbits.IREN = 0; //No IR translation
        U1MODEbits.RTSMD = 0; //Simplex Mode for RTS pin
        U1MODEbits.WAKE = 0; //No wakeup (we dont sleep)
        U1MODEbits.LPBACK = 0; //No Loop Back
        U1MODEbits.ABAUD = 0; //No Auto baudrate detection (would require sending '55')

        //important settings:
        U1MODEbits.UEN = 0; //TX,RX enabled, CTS,RTS not (no flow control)
        U1MODEbits.BRGH = 0; // 16 clocks per bit period (Standard Speed Mode)
        U1MODEbits.PDSEL = 0b00; //No parity
        U1MODEbits.STSEL = 0; // 1 Stop bit

        //set the baud rate
        
        if (baudrate == 115200){
            U1BRG = BRGVAL(baudrate) + 1;
        } else {
            U1BRG = BRGVAL(baudrate);
        }
        
        //generate a TX interrupt when the transmit buffer becomes empty
        U1STAbits.UTXISEL1 = 1;
        U1STAbits.UTXISEL0 = 0;

        U1STAbits.UTXINV = 0; //TX line idle state is 1 (high)

        //don't send a transmit sync break bit (start bit followed by 12 0's followed by stop)
        U1STAbits.UTXBRK = 0;

        U1STAbits.UTXEN = 0; //Disable transmit operations for now

        U1STAbits.URXISEL = 0; //Create an RX interrupt on every bit received

        U1STAbits.ADDEN = 0; //Address Detect Disabled (not useful unless 9-bit mode)

        //TX and RX mid range interrupt priority (no urgent reason)
        IPC3 &= ~(0b111); //clear last 3 bits
        IPC3 |= 0b100;
        IPC2 &= ~(0xf000); //clear upper 4 bits
        IPC2 |= (0b100 << 12);

        IFS0bits.U1TXIF = 0; // Clear the Transmit Interrupt Flag
        IFS0bits.U1RXIF = 0; // Clear the receive Interrupt Flag

        IEC0bits.U1RXIE = 1; // Enable receive Interrupts
        U1MODEbits.UARTEN = 1; // And turn the peripheral on
        U1STAbits.UTXEN = 1; //enable transmit operations (must come after the uart enable)
    } else if (interface == 2) { //we dont enable receive interrupts here as we dont need them
        U2MODEbits.UARTEN = 0; //Disable UART while we're configuring it
        U2MODEbits.USIDL = 0; //Keep interface on in idle mode
        U2MODEbits.IREN = 0; //No IR translation
        U2MODEbits.RTSMD = 0; //Simplex Mode for RTS pin
        U2MODEbits.WAKE = 0; //No wakeup (we dont sleep)
        U2MODEbits.LPBACK = 0; //No Loop Back
        U2MODEbits.ABAUD = 0; //No Auto baudrate detection (would require sending '55')

        //important settings:
        U2MODEbits.UEN = 0; //TX,RX enabled, CTS,RTS not (no flow control)
        U2MODEbits.BRGH = 0; // 16 clocks per bit period (Standard Speed Mode)
        U2MODEbits.PDSEL = 0b00; //No parity
        U2MODEbits.STSEL = 0; // 1 Stop bit

        if (baudrate == 115200){
            U2BRG = BRGVAL(baudrate) + 1;
        } else {
            U2BRG = BRGVAL(baudrate);
        }

        //generate a TX interrupt when the transmit buffer becomes empty
        U2STAbits.UTXISEL1 = 1;
        U2STAbits.UTXISEL0 = 0;

        U2STAbits.UTXINV = 0; //TX line idle state is 1 (high)

        //don't send a transmit sync break bit (start bit followed by 12 0's followed by stop)
        U2STAbits.UTXBRK = 0;

        U2STAbits.UTXEN = 0; //Disable transmit operations for now

        U2STAbits.URXISEL = 0; //Create an RX interrupt on every bit received

        U2STAbits.ADDEN = 0; //Address Detect Disabled (not useful unless 9-bit mode)

        // Mid Range Interrupt Priority level for RX and TX, no urgent reason
        IPC7 &= ~(0xff00); //clear upper 8 bits
        IPC7 |= (0b100 << 12);
        IPC7 |= (0b100 << 8);

        IFS1bits.U2TXIF = 0; // Clear the Transmit Interrupt Flag
        IFS1bits.U2RXIF = 0; // Clear the receive Interrupt Flag

        U2MODEbits.UARTEN = 1; // And turn the peripheral on
        U2STAbits.UTXEN = 1; //enable transmit operations (must be called after uart enable)
    }
}

void sendTXData(uint8_t interface, uint8_t* data, uint16_t data_length)
{
    uint16_t i = 0;
    
    if (interface == 1) {
        while(i < data_length){
            if (!U1STAbits.UTXBF){
                U1TXREG = data[i];
                i++;
            }
        }
    } else if (interface == 2) {
         while(i < data_length){
            if (!U2STAbits.UTXBF){
                U2TXREG = data[i];
                i++;
            }
        }
    }
}