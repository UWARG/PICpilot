#include <p33FJ256GP710.h>
#include "Simulator.h"

unsigned char rx2Data[RAW_PACKET_BUFFER_SIZE];
unsigned char txData[SEND_PACKET_SIZE];

int currSendByte = 0;
int txSendData = 0;
int payloadPos = 0;
static int packetStatus = 0;
int newPacketReceived = 0;

struct ImuData *simData;
struct ImuData *rxData;

struct ImuData *getSimData(void){
    return simData;
}
/*
 char x[30];
sprintf(&x, "FIRST: %02X", rxByte);
debug(&x);*/
void setSimData(void){
    simData = (struct ImuData*)rx2Data;
}

void setTxPacket(char sendStat, int throttle, int pitch, int roll, int yaw)
{
    if (txSendData == 0){
        int tempInt[4];
        tempInt[0] = throttle;
        tempInt[1] = pitch;
        tempInt[2] = roll;
        tempInt[3] = yaw;

        int fillCount = 0;
        txData[fillCount] = 0x45; // start delimeter
        fillCount++;
        int j = 0;
        for (j = 0; j < 4; j++){
            union byteint bi;
            bi.i = tempInt[j];
            int index = fillCount;
            int k = 0;
            for (k = index; k < (index + 2); k++){
                txData[k] = bi.b[k - index];
                fillCount++;
            }
        }
        txData[fillCount] = sendStat; //sendStatus
        fillCount++;
        txData[fillCount] = 0x4A; //end delimeter
        fillCount++;
        txSendData = 1;
    }
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
            payloadPos++;
            //if the end delimiter is seen, save the packet, reset, and flag
            //that a new packet is ready for processing
            if (payloadPos >= 31){//rxByte == END_DELIMITER){ //the struct is 32 bytes big, so the array index max is 31
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

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void) {
    // Short circuit if nothing in the staging area yet
    if ( txSendData == 0 ) {
        IFS0bits.U1TXIF = 0;
        return;
    }

    U1TXREG = txData[currSendByte];
    currSendByte++;
    if(currSendByte >= SEND_PACKET_SIZE){
        currSendByte = 0;
        txSendData = 0;
    }
    IFS0bits.U1TXIF = 0;
}