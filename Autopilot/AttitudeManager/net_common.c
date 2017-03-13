/**
 * net_outbound.c
 */

#include "net.h"

#include "main.h"
#include "UART2.h"

// Initialize the data link
int initDataLink(void) {
    InitUART2();
    IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupt
    IEC1bits.U2RXIE = 1;        // Enable RX Interrupt

    return 0;
}

int checkDataLinkConnection(){
    //TODO: Should actually implement this properly
    return 1;
}
