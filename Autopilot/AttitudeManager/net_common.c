/**
 * net_outbound.c
 */

#include "net.h"

#include "p33FJ256GP710.h"
#include "UART2.h"


// Initialize the data link
int initDataLink(void) {
    InitUART2();
    IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupt
    IEC1bits.U2RXIE = 1;        // Enable RX Interrupt

    return 0;
}
