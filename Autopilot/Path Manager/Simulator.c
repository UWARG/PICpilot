#include <p33FJ256GP710.h>
#include "Simulator.h"

unsigned char rx2Data[RAW_PACKET_BUFFER_SIZE];

int currSendByte = 0;
int txSendData = 0;
int payloadPos = 0;
static int packetStatus = 0;
int newPacketReceived = 0;

struct GpsData *simData;
struct GpsData *rxData;

struct GpsData *getSimData(void){
    simData->latitude = 62.453;
    simData->longitude = 45.8465;
    return simData;
}
/*
 char x[30];
sprintf(&x, "FIRST: %02X", rxByte);
debug(&x);*/
void setSimData(void){
    simData = (struct GpsData*)rx2Data;
    char x[30];
    sprintf(x, "11FIRST: %02X", rx2Data[0]);
    debug(&x);

    char x1[30];
    sprintf(x1, "22FIRST: %02X", rx2Data[28]);
    debug(&x1);
}

char getRxPacketStatus(){
    if (newPacketReceived == 1){
        newPacketReceived = 0;
        return 1;
    }
    else
        return 0;
}

void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void) {

    //If Overrun error occurs then reset
    if (U1STAbits.OERR == 1){
        debug("Overrun ERROR");
        U1STAbits.OERR = 0;
        IFS0bits.U1RXIF = 0;
        return;
    }

    //Continue to empty the UART Receive buffer if there's something to process
    //This will prevent Overrun errors from occurring
    while(U1STAbits.URXDA == 1){

       //Get the byte from the receive register
       unsigned char rxByte = U1RXREG;

       //if a packet hasn't been started (based on seeing te starting delimiter
       if (!packetStatus){
           //reset the dataPacket index
            payloadPos = 0;

            if (rxByte == START_DELIMITER){
                packetStatus = 1;
            }
            else if (rxByte == END_DELIMITER){
                packetStatus = 0;
            }
        }

        //if a dataPacket has already been started
        if(packetStatus){
            //still receiving parts of packet, so continue assembly
//            ((unsigned char*)rxData)[payloadPos] = rxByte;
            rx2Data[payloadPos] = rxByte;
    char x1[30];
    sprintf(x1, "FIRST[%d]: %02X",payloadPos, rxByte);
    debug(&x1);
            payloadPos++;
            //if the end delimiter is seen, save the packet, reset, and flag
            //that a new packet is ready for processing
            if (payloadPos >= RAW_PACKET_BUFFER_SIZE-1){//rxByte == END_DELIMITER){ //the struct is 32 bytes big, so the array index max is 31
                setSimData();//rxData);
                packetStatus = 0;
                payloadPos = 0;
                newPacketReceived = 1;
            }
        }
    }

    //reset the interrupt flag because I'm not a newb
    IFS0bits.U1RXIF = 0;
}