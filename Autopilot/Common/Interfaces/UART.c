/**
 * @file UART.c
 * @author Serge Babayan
 * @date February 14, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "UART.h"
#include <p33FJ256GP710A.h>
#include "../clock.h"
#include "../Utilities/ByteQueue.h"

/**
 * The chip requires the baud rate register to be set to a value that corresponds
 * to the processor frequency. The formula can be found on the data sheet. This
 * takes an ordinary baud rate value in and returns that
 */
#define BRGVAL(bd) (((CLOCK_FREQUENCY/bd)/16)-1)

static char uart1_initialized = 0;
static char uart2_initialized = 0;

static ByteQueue uart1_rx_queue;
static ByteQueue uart1_tx_queue;
static ByteQueue uart2_rx_queue;
static ByteQueue uart2_tx_queue;

void initUART(unsigned char interface, unsigned long int baudrate, unsigned int initial_buffer_size, unsigned int max_buffer_size)
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
        U1MODEbits.URXINV = 0; //IdleState = 1  (for dsPIC)
        U1MODEbits.BRGH = 0; // 16 clocks per bit period (Standard Speed Mode)
        U1MODEbits.PDSEL = 0b00; //No parity
        U1MODEbits.STSEL = 0; // 1 Stop bit

        //set the baud rate
        U1BRG = BRGVAL(baudrate);

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
        IEC0bits.U1TXIE = 1; // Enable Transmit Interrupts
        IFS0bits.U1RXIF = 0; // Clear the receive Interrupt Flag
        IEC0bits.U1RXIE = 1; // Enable receive Interrupts

        U1MODEbits.UARTEN = 1; // And turn the peripheral on
        U1STAbits.UTXEN = 1; //enable transmit operations
        
        initBQueue(&uart1_rx_queue, initial_buffer_size, max_buffer_size);
        initBQueue(&uart1_tx_queue, initial_buffer_size, max_buffer_size);
        uart1_initialized = 1;
    } else if (interface == 2) {
        U2MODEbits.UARTEN = 0; //Disable UART while we're configuring it
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
        U2MODEbits.PDSEL = 0b00; //No parity
        U2MODEbits.STSEL = 0; // 1 Stop bit

        //set the baud rate
        U2BRG = BRGVAL(baudrate);

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
        IEC1bits.U2TXIE = 1; // Enable Transmit Interrupts
        IFS1bits.U2RXIF = 0; // Clear the receive Interrupt Flag
        IEC1bits.U2RXIE = 1; // Enable receive Interrupts

        U2MODEbits.UARTEN = 1; // And turn the peripheral on
        U2STAbits.UTXEN = 1; //enable transmit operations

        initBQueue(&uart2_rx_queue, initial_buffer_size, max_buffer_size);
        initBQueue(&uart2_tx_queue, initial_buffer_size, max_buffer_size);
        uart2_initialized = 1;
    }
}

void quoueTXData(unsigned char interface, unsigned char* data, unsigned int data_length)
{
    unsigned int i;
    if (interface == 1 && uart1_initialized == 1) {
        for (i = 0; i < data_length; i++) {
            pushBQueue(&uart1_tx_queue, data[i]);
        }
        
        if (U1STAbits.TRMT){ //if the transmit buffer is empty and register (no sending)
            U1TXREG = popBQueue(&uart1_tx_queue); //trigger a send
        }  
    } else if (interface == 2 && uart2_initialized == 1) {
        for (i = 0; i < data_length; i++) {
            pushBQueue(&uart2_tx_queue, data[i]);
        }
        
        if (U2STAbits.TRMT){ //if the transmit buffer is empty and register (no sending)
            U2TXREG = popBQueue(&uart2_tx_queue); //trigger a send
        } 
    }
}

unsigned char readRXData(unsigned char interface)
{
    if (interface == 1) {
        return popBQueue(&uart1_rx_queue);
    } else if (interface == 2) {
        return popBQueue(&uart2_rx_queue);
    }
    return -1;
}

unsigned int getTXSpace(unsigned char interface){
    if (interface == 1) {
        return getBQueueSpace(&uart1_tx_queue);
    } else if (interface == 2) {
        return getBQueueSpace(&uart2_tx_queue);
    }
    return -1;
}

/**
 * Interrupt called whenever the TX buffer becomes empty
 */
void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void)
{
    //while the TX buffer isn't full and we have data to send 
    while (!U2STAbits.UTXBF && uart2_tx_queue.size != 0) {
        U2TXREG = popBQueue(&uart2_tx_queue);
    }
    IFS1bits.U2TXIF = 0;
}

/**
 * Interrupt called when there is at least 1 character available to read from the rx buffer
 */
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
{
    //while the rx buffer has characters available to read
    while (U2STAbits.URXDA) {
        pushBQueue(&uart2_rx_queue, U2RXREG);
    }
}

/**
 * Interrupt called whenever the TX buffer becomes empty
 */
void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
    //while the TX buffer isn't full and we have data to send 
    while (!U1STAbits.UTXBF && uart1_tx_queue.size != 0) {
        U1TXREG = popBQueue(&uart1_tx_queue);
    }
    IFS0bits.U1TXIF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    //while the rx buffer has characters available to read
    while (U1STAbits.URXDA) {
        pushBQueue(&uart1_rx_queue, U1RXREG);
    }
}