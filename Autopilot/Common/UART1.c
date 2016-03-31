
#include <p33FJ256GP710.h>
#include "UART1.h"

#define RAW_PACKET_BUFFER_SIZE 100
#define START_DELIMITER '('
#define END_DELIMITER ')'

unsigned char rxData[RAW_PACKET_BUFFER_SIZE];
char dataPacket[RAW_PACKET_BUFFER_SIZE];

static int payloadPos = 0;
static int packetStatus = 0;
int newPacketReceived = 0;


unsigned char* getData(){
    return dataPacket;
}

void setData(char* msg){

    //Clear the previous dataPacket
    *dataPacket = "";

    //Identify the length of the message
    unsigned int mLength = 0;
    while (msg[mLength] != '\0' && mLength < RAW_PACKET_BUFFER_SIZE - 1){
        mLength++;
    }

    //Fill the dataPacket with the new message
    unsigned int i = 0;
    for (i = 0; i < mLength; i++){
        dataPacket[i] = msg[i];
    }
    printf("Q");
    UART1_SendString(dataPacket);
}

//Identifies whether or not there's new information to be processed
int getRxPacketStatus(){
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
        *rxData = "";
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
                *rxData = "";
            }
            else if (rxByte == END_DELIMITER){
                packetStatus = 0;
                *rxData = "";
            }
        }

        //if a dataPacket has already been started
        else{

            //if the end delimiter is seen, save the packet, reset, and flag
            //that a new packet is ready for processing
            if (rxByte == END_DELIMITER){
                setData(&rxData);
                packetStatus = 0;
                payloadPos = 0;
                *rxData = "";
                newPacketReceived = 1;
            }

            //still receiving parts of packet, so continue assembly
            else{
                rxData[payloadPos] = rxByte;
                payloadPos++;
            }
        }
    }

    //reset the interrupt flag because I'm not a newb
    IFS0bits.U1RXIF = 0;
}

void InitUART1()
{
	// This is an EXAMPLE, so brutal typing goes into explaining all bit sets

	// The HPC16 board has a DB9 connector wired to UART2, so we will
	// be configuring this port only

	// configure U2MODE
	U1MODEbits.UARTEN = 0;	// Bit15 TX, RX DISABLED, ENABLE at end of func
	//U2MODEbits.notimplemented;	// Bit14
	U1MODEbits.USIDL = 0;	// Bit13 Continue in Idle
	U1MODEbits.IREN = 0;	// Bit12 No IR translation
	U1MODEbits.RTSMD = 0;	// Bit11 Simplex Mode
	//U2MODEbits.notimplemented;	// Bit10
	U1MODEbits.UEN = 0;		// Bits8,9 TX,RX enabled, CTS,RTS not
	U1MODEbits.WAKE = 0;	// Bit7 No Wake up (since we don't sleep here)
	U1MODEbits.LPBACK = 0;	// Bit6 No Loop Back
	U1MODEbits.ABAUD = 0;	// Bit5 No Autobaud (would require sending '55')
	U1MODEbits.URXINV = 0;	// Bit4 IdleState = 1  (for dsPIC)
	U1MODEbits.BRGH = 0;	// Bit3 16 clocks per bit period
	U1MODEbits.PDSEL = 0b00;	// Bits1,2 8bit, No Parity
	U1MODEbits.STSEL = 0;	// Bit0 One Stop Bit

        //***NOTE That Mitch Changed this code and the Baud Rate can be automatically calculated at the top
	// Load a value into Baud Rate Generator.  Example is for 115200.
	// See section 19.3.1 of datasheet.
	//  U1BRG = (Fcy/(16*BaudRate))-1
	//  U1BRG = (38707200/(16*115200))-1
	//  U1BRG = 20

        //UART Baud Rate Calculated Values...
        //9600 =
        //19200 =
        //38400 = 63
        //115200 = 20
        U1BRG = 20;//Baud_Rate;	// 40Mhz osc, ___ Baud

	// Load all values in for U1STA SFR
	U1STAbits.UTXISEL1 = 0;	//Bit15 Int when Char is transferred (1/2 config!)
	U1STAbits.UTXINV = 0;	//Bit14 N/A, IRDA config
	U1STAbits.UTXISEL0 = 0;	//Bit13 Other half of Bit15
	//U2STAbits.notimplemented = 0;	//Bit12
	U1STAbits.UTXBRK = 0;	//Bit11 Disabled
	U1STAbits.UTXEN = 0;	//Bit10 TX pins controlled by periph
	U1STAbits.UTXBF = 0;	//Bit9 *Read Only Bit*
	U1STAbits.TRMT = 0;	//Bit8 *Read Only bit*
	U1STAbits.URXISEL = 0;	//Bits6,7 Int. on character recieved
	U1STAbits.ADDEN = 0;	//Bit5 Address Detect Disabled
	U1STAbits.RIDLE = 0;	//Bit4 *Read Only Bit*
	U1STAbits.PERR = 0;		//Bit3 *Read Only Bit*
	U1STAbits.FERR = 0;		//Bit2 *Read Only Bit*
	U1STAbits.OERR = 0;		//Bit1 *Read Only Bit*
	U1STAbits.URXDA = 0;	//Bit0 *Read Only Bit*

	IPC7 = 0x4400;	// Mid Range Interrupt Priority level, no urgent reason

	IFS0bits.U1TXIF = 0;	// Clear the Transmit Interrupt Flag
	IEC0bits.U1TXIE = 0;	// Enable Transmit Interrupts
	IFS0bits.U1RXIF = 0;	// Clear the Recieve Interrupt Flag
	IEC0bits.U1RXIE = 1;	// Enable Recieve Interrupts

	U1MODEbits.UARTEN = 1;	// And turn the peripheral on

	U1STAbits.UTXEN = 1;
	// I think I have the thing working now.


        /* wait at least 104 usec (1/9600) before sending first char */
        int i = 0;
        while (i < 4160)
        {
            Nop();
            i++;
        }
}

void UART1_SendChar(char data)
{
    U1TXREG = data;
    while(U1STAbits.TRMT == 0);
    U1STAbits.TRMT = 0;
}

void UART1_SendString(char *s)
{
    do
    {
        UART1_SendChar(*s);
        s++;
        //Delay10TCYx(250);
    }while(*s != 0);

          UART1_SendChar('\n');
}
