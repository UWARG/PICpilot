#include "../Common/Common.h"
#include "I2C.h"
void initI2C()
{
    //Initializes all I2C communications registers


    //Disable module to begin
    I2C2CONbits.I2CEN = 0;
    //Do not disable module in Idle mode (Power saving)
    I2C2CONbits.I2CSIDL = 1;
    //7 bit address mode
    I2C2CONbits.A10M = 0;
    //Intelligent Periphiral Interface Management disabled
    I2C2CONbits.IPMIEN = 0;
    //Slew Rate control disabled for 100Khz
    I2C2CONbits.DISSLW = 1;
    //Do NOT use SMBus voltage configuration
    I2C2CONbits.SMEN = 0;

    ///I2C1BRG  - BAUD RATE GENERATOR
    ///MPL3115A2 requires Max 4MHz
    I2C2BRG = 19;//0x09; //385khz clock rate; FCY = 4MHZ

    //Enable the I2C module
    I2C2CONbits.I2CEN = 1;

//    //Clear bus send stop condition
    I2C2CONbits.PEN = 1;
    I2CIdle();

}

char checkDevicePresence(char devAddress, char reg){
    char connected = 0;
    int count = 0;
    char data = 0;

    I2C2CONbits.SEN = 1;  //Send Start condition
    I2CIdle();
    //SET Slave Address & write (Address shifted one bit left and then the write(0) bit is added)
    I2C2TRN = devAddress << 1; //If reading, the read process is specified after the dummy bytes.
    I2CIdle();
    I2C2TRN = reg;  //Then after it is free, write the local address.
    I2CIdle(); //Wait until acknowledge is sent from the slave
    I2C2CONbits.RSEN = 1; //Resend the start condition
    I2CIdle(); //Wait until acknowledge is sent from the slave
    I2C2TRN = (devAddress << 1) + 1; //Shift and add the read bit(1) - Prep for restart
    I2CIdle(); //Wait until acknowledge is sent from the slave
    ///THE MESSAGE FROM THE SLAVE IS SENT HERE
    I2C2CONbits.RCEN = 1; //Enable receive mode
    I2CIdle(); //Wait until all 8 bits have been acquired
    while (I2C2STATbits.RBF != 1 && count < 0x0FFF)count++;
    data = I2C2RCV;
    //Send back a NACK
    I2C2CONbits.ACKDT = 1; //Send NACK
    I2C2CONbits.ACKEN = 1; //Start the acknowledge sequence
    if (data){
        I2CIdle(); //Wait until done
    }
    if (data){
        connected = 1;
    }
    else{
        connected = 0;
    }
    
    I2C2CONbits.PEN = 1;
    while((I2C2CON & 0x1F ) || I2C2STATbits.TRSTAT == 1);

    return connected;
}

char sendMessage(char devAddress, char address, char* data, char length, char rw)
{
    char rData = 0;
    I2CIdle();
    I2C2CONbits.SEN = 1;  //Send Start condition
    I2CIdle();

    //SET Slave Address & write (Address shifted one bit left and then the write(0) bit is added)
    I2C2TRN = devAddress << 1; //If reading, the read process is specified after the dummy bytes.

    if (rw == READ) //If in reading mode
    {
        rData = readMessage(devAddress, address);
    }
    else //Otherwise go into writing mode
    {
       writeMessage(address,data, length);
    }

    I2CIdle();
    I2C2CONbits.PEN = 1; //Send Stop condition

    I2CIdle();

    return rData;

}

char readMessage(char devAddress, char address)
{
    I2CIdle();
    I2C2TRN = address;  //Then after it is free, write the local address.
    I2CIdle(); //Wait until acknowledge is sent from the slave
    I2C2CONbits.RSEN = 1; //Resend the start condition
    I2CIdle(); //Wait until acknowledge is sent from the slave
    I2C2TRN = (devAddress << 1) + 1; //Shift and add the read bit(1) - Prep for restart
    I2CIdle(); //Wait until acknowledge is sent from the slave

    ///THE MESSAGE FROM THE SLAVE IS SENT HERE
    I2C2CONbits.RCEN = 1; //Enable receive mode
    I2CIdle(); //Wait until all 8 bits have been acquired
    while (I2C2STATbits.RBF != 1);
    char data = I2C2RCV;
    //Send back a NACK
    I2C2CONbits.ACKDT = 1; //Send NACK
    I2C2CONbits.ACKEN = 1; //Start the acknowledge sequence
    I2CIdle(); //Wait until done
    return data;


}
void writeMessage(char address, char* data, char length)
{
    I2CIdle();
    I2C2TRN = address;  //Then after it is free, write the address.

//    I2CIdle();//Check until transmition was completed

    //Write each byte of data
    int i = 0;
    for(i = 0; i < length; i++)
    {
        I2CIdle();//Check until transmition was completed
        I2C2TRN = (char)data[i];
    }

}
