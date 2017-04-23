#include "UART.h"
#include "Logger.h"
#include "Peripherals.h"
#include "p24F16KA101.h"
#include <xc.h>
#include <math.h>
#include "Config.h"

/**
 * Frequency of the oscillator attached to the chip. Should be 8Mhz
 */
#define Fosc	(8000000)

/**
 * MIPS of the processor. x4 because we've enabled PLL on the clock. /2 because of how
 * pic chips operate. This is the effective frequency of the chip
 */
#define Fcy		(Fosc*4/2)	// w.PLL (Instruction Per Second)

//#define Fsck	100000		// 400kHz I2C
//#define I2C_BRG	((Fcy/2/Fsck)-1)

#define NMEA_MAX_PACKET_LENGTH 90

// FBS
#pragma config BWRP = OFF               // Table Write Protect Boot (Boot segment may be written)
#pragma config BSS = OFF                // Boot segment Protect (No boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Code Flash Write Protection bit (General segment may be written)
#pragma config GCP = OFF                // General Segment Code Flash Code Protection bit (No protection)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Select (Primary oscillator with PLL module (HS+PLL, EC+PLL))
#pragma config IESO = ON                // Internal External Switch Over bit (Internal External Switchover mode enabled (Two-Speed Start-up enabled))

// FOSC
#pragma config POSCMOD = HS             // Primary Oscillator Configuration bits (HS oscillator mode selected)
#pragma config OSCIOFNC = OFF           // CLKO Enable Configuration bit (CLKO output signal is active on the OSCO pin)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8 MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary oscillator configured for high-power operation)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (WDT prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected; windowed WDT disabled)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default location for SCL1/SDA1 pins)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
#pragma config MCLRE = ON               // MCLR Pin Enable bit (MCLR pin enabled; RA5 input pin disabled)

// FICD
#pragma config ICS = PGx2               // ICD Pin Placement Select bits (PGC2/PGD2 are used for programming and debugging the device)

// FDS
#pragma config DSWDTPS = DSWDTPSF       // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses LPRC as reference clock)
#pragma config RTCOSC = SOSC            // RTCC Reference Clock Select bit (RTCC uses SOSC as reference clock)
#pragma config DSBOREN = OFF            // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR disabled in Deep Sleep)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer Enable bit (DSWDT disabled)


unsigned char data = 0x00;
unsigned char newDataFlag = 0x00;
unsigned int byteCount = 0;
unsigned char stringType = 0; //0 = Unknown, 1 = GGA, 2 = VTG
unsigned char headerData[6];
unsigned char bufferData[NMEA_MAX_PACKET_LENGTH];
unsigned char gga[NMEA_MAX_PACKET_LENGTH];
unsigned char vtg[NMEA_MAX_PACKET_LENGTH];

unsigned char readLock = 0;
unsigned char newData = 0;
unsigned char sp[100];
unsigned char apiString[100];
unsigned int spIndex;
unsigned char airspeed[4];
unsigned char altitude[5];
unsigned char stringID[6];
unsigned char checkSum;
unsigned char headerCheckSum;
unsigned int nmeaStringID;
unsigned int comma;
unsigned int dataValid;
unsigned char dataI2C_1;
unsigned char dataI2C_2;

char rawTime[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
char rawLatitude[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
char rawLongitude[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
char rawSatellites[3] = {0, 0, 10};
char rawAltitude[8] = {0, 0, 0, 0, 0, 0, 0, 0};
char rawHeading[6] = {0, 0, 0, 0, 0, 0};
char rawGroundSpeed[8] = {0, 0, 0, 0, 0, 0, 0, 0};
char latitudeNS = 0;
char longitudeEW = 0;
char positionFix = 0;

int configGPS(void);
int identifyString(char *stringArray);
char readGPSData(void);
int ParseGGA(void);
int ParseVTG(void);
void processData(void);
char HexConvert(unsigned char ascciSymbol);
char CharConvert(unsigned int checkSumHalf);
int Delay_ms(unsigned int millisec);
int Delay_half_us(unsigned int microsec);

struct GPSData {
    long double latitude;  //8 Bytes
    long double longitude; //8 Bytes
    float time;     //4 Bytes
    float speed;
    int altitude;
    int heading;
    char satellites;    //1 Byte
    char positionFix;
};
struct GPSData gpsData;

/**
 * Initializes Timer4 as a 1ms, 16-bit timer
 */
void initTimer4(){
    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TCS = 0; // Select internal instruction cycle clock
    T1CONbits.TGATE = 0; // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b10; // Select 1:8 Prescaler
    TMR1 = 0x00; // Clear timer register
    PR1 = 250; // Load the period value. Trigger every ms
    IPC0bits.T1IP = 0x01; // Set Timer 4 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer 4 Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer 4 interrupt
    T1CONbits.TON = 1; // Start Timer
}

volatile uint32_t time_ms = 0;
/**
 * Timer4 interrupt. Executed every ms
 */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void){
    time_ms += 1;
    IFS0bits.T1IF = 0;
}


void delay(uint32_t ms) {
    uint32_t start_time = time_ms;
    while(time_ms - start_time < ms){

    };
}

/******************************* Interrupts ************************************/
void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void) {
    //if(SPI1BUF == 1) PORTFbits.RF4 = 1;
    //else PORTFbits.RF4 = 0;
    //        PORTAbits.RA6 = 1;

    IFS0bits.SPI1IF = 0; // Clear the Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
    while (U1STAbits.URXDA) {
        data = U1RXREG;

        if (data == '$') {
            //Beginning of Packet
            stringType = 0;
            byteCount = 0;
        } else if (data == 0x0A) {
            //End of Packet
            stringType = 0;
            byteCount = 0;
            newData = 1;
        } else if (stringType == 1){
            if (!readLock)
                gga[byteCount] = data;
        } else if (stringType == 2){
            if (!readLock)
                vtg[byteCount] = data;
        }


        if (byteCount <= 5){
            headerData[byteCount] = data;
        }

        //Identify the packet type once the header has been read
        if (byteCount == 5) {
            stringType = identifyString(&headerData);
        }
        byteCount++;
    }
    IFS0bits.U1RXIF = 0; // Clear the Interrupt Flag
}

//******************************Functions************************************


void initSPI(){
    //initialize intercom SPI port
    TRISBbits.TRISB12 = 0; //SPI data out
    TRISBbits.TRISB13 = 0; //SPI clk out
    TRISBbits.TRISB15 = 0; //SPI slave select out
    PORTBbits.RB15 = 1; //slave disabled

    SPI1BUF = 0;
    IFS0bits.SPI1IF = 0; // Clear the Interrupt Flag
    IEC0bits.SPI1IE = 0; // Disable the Interrupt
    //SPI1CON1 Register Settings
    SPI1CON1bits.DISSCK = 0; // Internal Serial Clock is Enabled
    SPI1CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
    SPI1CON1bits.MODE16 = 1; // Communication is byte-wide (8 bits)
    SPI1CON1bits.SMP = 0; // Input data is sampled at the middle of data
    // output time
    SPI1CON1bits.CKE = 0; // Serial output data changes on transition
    // from Idle clock state to active clock state
    SPI1CON1bits.CKP = 0; // Idle state for clock is a low level; active
    // state is a high level
//    SPI1CON1bits.SSEN = 1;	// Enable slave select pin - don't uncomment this UART won't work
    SPI1CON1bits.SPRE = 0b110;
    SPI1CON1bits.PPRE = 0b01;
    SPI1CON1bits.MSTEN = 1; // Master mode Enabled
    SPI1STATbits.SPIROV = 0; // SPI recieve overflow
    SPI1STATbits.SPIEN = 1; // Enable SPI module

    // Interrupt Controller Settings
    IFS0bits.SPI1IF = 0; // Clear the Interrupt Flag
    IEC0bits.SPI1IE = 1; // Enable the Interrupt
}

int configGPS(void) {
    //check baud rate first
    debug("Checking to see if GPS has to be reconfigured..");
//    int j;
//    int at_9600 = 0;
//    for (j = 0; j <= 4; j++) {
//        while (U1STAbits.URXDA != 1) {
//            ;
//        }
//        data = U1RXREG;
//        if (U1STAbits.FERR == 1) {
//            at_9600 = 1;
//            //			PORTAbits.RA6 = 1;
//            Delay_ms(2000);
//
//        }
//    }
    
    if ( 1) {
        debug("Reconfiguring GPS..");
        U1BRG = 103;
        int i;
        char changeBaud[] = {'$', 'P', 'M', 'T', 'K', '2', '5', '1', ',', '1', '1', '5', '2', '0', '0', '*', '1', 'F', 13, 10}; // changes UART speed to 115200 - for loop (0 <=19)
        char changeStrings[] = {'$', 'P', 'M', 'T', 'K', '3', '1', '4', ',', '0', ',', '0', ',', '1', ',', '1', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', ',', '0', '*', '2', 'C', 13, 10}; //instructs gps to only send proper two strings - for loop (0 <=50)
        char changeFreeze[] = {'$', 'P', 'M', 'T', 'K', '3', '9', '7', ',', '0', '*', '2', '3', 13, 10}; // (0 <=16) removes freezing of speed
        char changeRate[] = {'$', 'P', 'M', 'T', 'K', '2', '2', '0', ',', '1', '0', '0', '*', '2', 'F', 13, 10}; // changes update rate(0 <=16)
        char changeDGPS[] = {'$', 'P', 'M', 'T', 'K', '3', '0', '1', ',', '2', '*', '2', 'E', 13, 10}; //Enables WAAS(Wide Area Augmentation System) DGPS

//        for (i = 0; i <= 19; i++) {
//            while (U1STAbits.UTXBF == 1) {;}
//            U1TXREG = changeBaud[i];
//        }
        delay(200);
        U1BRG = 8;

        for (i = 0; i <= 50; i++) {
            while (U1STAbits.UTXBF == 1) {;}
            U1TXREG = changeStrings[i];
        }
        delay(200);


        for (i = 0; i <= 14; i++) {
            while (U1STAbits.UTXBF == 1) {;}
            U1TXREG = changeFreeze[i];
        }

        delay(200);

        for (i = 0; i <= 16; i++) {
            while (U1STAbits.UTXBF == 1) {;}
            U1TXREG = changeRate[i];
        }
        delay(200);

        for (i = 0; i <= 14; i++) {
            while (U1STAbits.UTXBF == 1) {;}
            U1TXREG = changeDGPS[i];
        }
        delay(200);

        U1STAbits.OERR = 0;
        debug("GPS configured");
    } else{
        debug("No gps configuration required as battery was plugged in..");
    }

    return 0;
}

int identifyString(char *stringArray) {
    headerCheckSum = 0x00;
    char nmeaString = 0;
    int i = 1;
    for (i = 1; i < 6; i++) {
        headerCheckSum ^= stringArray[i];
    }
    //The checkSum of "GPGGA" evaluates to: 0x56
    //The checkSum of "GPVTG" evaluates to: 0x52
    if (headerCheckSum == 0x56) {
        nmeaString = 1;
        PORTAbits.RA6 = 1;
    } //GGA
    if (headerCheckSum == 0x52) {
        nmeaString = 2;
        PORTAbits.RA6 = 0;
    } //VTG
    return nmeaString;
}

char readGPSData(void) {
    char integrity = 1;
    readLock = 1;
    integrity &= ParseGGA();
    integrity &= ParseVTG();

//            	while(U2STAbits.UTXBF == 1){;}
//    	U2TXREG = integrity + 48;
    //    	while(U2STAbits.UTXBF == 1){;}
    //	U2TXREG = 13;
        readLock = 0;
    return integrity;
}

int ParseGGA(void) {
    dataValid = 1;
    comma = 0;
    checkSum = 0x56;
    int i = 0;
    int j = 6;
    //struct GPSData tmpData;

    while (gga[j] != '*') {
        char numData = HexConvert(gga[j]);
        if (gga[j] != '*') {
            checkSum = checkSum ^ gga[j];
        }
        if (gga[j] == ',') {
            comma++;
            i = 0;
            //			while(U2STAbits.UTXBF == 1){;}
            //			U2TXREG = 44;
        }
        if ((comma == 1) && (i != 0)) {
            rawTime[i] = numData;

            //			while(U2STAbits.UTXBF == 1){;}
            //			U2TXREG = numData;
        }
        if ((comma == 2) && (i != 0)) {
            rawLatitude[i] = numData;
            //			while(U2STAbits.UTXBF == 1){;}
            //			U2TXREG = data;

        }
        if ((comma == 3) && (i != 0)) {
            latitudeNS = gga[j];
//            			while(U2STAbits.UTXBF == 1){;}
//            			U2TXREG = gga[j];

        }
        if ((comma == 4) && (i != 0)) {
            rawLongitude[i] = numData;
            //			while(U2STAbits.UTXBF == 1){;}
            //			U2TXREG = data;

        }
        if ((comma == 5) && (i != 0)) {
            longitudeEW = gga[j];
//            			while(U2STAbits.UTXBF == 1){;}
//            			U2TXREG = gga[j];

        }
        if ((comma == 6) && (i != 0)) {
            positionFix = numData;
//            			while(U2STAbits.UTXBF == 1){;}
//            			U2TXREG = gga[j];

        }
        if ((comma == 7) && (i != 0)) {
            rawSatellites[i] = numData;
            //			while(U2STAbits.UTXBF == 1){;}
            //			U2TXREG = numData;
        }
        if ((comma == 9) && (i != 0)) {
            rawAltitude[i] = numData;
//            			while(U2STAbits.UTXBF == 1){;}
//            			U2TXREG = gga[j];
        }
        i++;
        j++;
    }

    // Carriage Return value
    //*********checksum************
    j++;
    char checkSumTemp = checkSum & 0xF0;
    checkSumTemp = checkSumTemp >> 4;
    if (HexConvert(gga[j++]) != checkSumTemp) {
        dataValid = 0;
    }

    //                      	while(U2STAbits.UTXBF == 1){;}
    //	U2TXREG = dataValid + 48;
    //    	while(U2STAbits.UTXBF == 1){;}
    //	U2TXREG = 13;

    checkSumTemp = checkSum & 0x0F;
    if (HexConvert(gga[j++]) != checkSumTemp) {
        dataValid = 0;
    }
    //*********checksum************
    return dataValid;
}

int ParseVTG(void) {
    dataValid = 1;
    checkSum = 0x52;
    comma = 0;
    int i = 0;
    int j = 6;

    while (vtg[j] != '*') {
        char numData = HexConvert(vtg[j]);
        if (vtg[j] != '*') {
            checkSum ^= vtg[j];
        }
        if (vtg[j] == ',') {
            comma++;
            i = 0;
        }
        if (comma == 1 && (i != 0)) {
            rawHeading[i] = numData;
//                                    while(U2STAbits.UTXBF == 1){;}
//            			U2TXREG = vtg[j];
        }
        if (comma == 7 && (i != 0)) {
            rawGroundSpeed[i] = numData;
//                                    while(U2STAbits.UTXBF == 1){;}
//            			U2TXREG = vtg[j];
        }
        i++;
        j++;
    }
    // Carriage Return value
    //*********checksum************
    j++;
    char checkSumTemp = checkSum & 0xF0;
    checkSumTemp = checkSumTemp >> 4;
    if (HexConvert(vtg[j++]) != checkSumTemp) {
        dataValid = 0;
    }

    checkSumTemp = checkSum & 0x0F;
    if (HexConvert(vtg[j++]) != checkSumTemp) {
        dataValid = 0;
    }
    //*********checksum************
    return dataValid;
}

void convertData(void) {
    //calculate time
    gpsData.time = (float) rawTime[1] * 100000;
    gpsData.time += (float) rawTime[2] * 10000;
    gpsData.time += (float) rawTime[3] * 1000;
    gpsData.time += (float) rawTime[4] * 100;
    gpsData.time += (float) rawTime[5] * 10;
    gpsData.time += (float) rawTime[6] * 1;
    //Decimal Point
    gpsData.time += (float) rawTime[8] * 0.1;
    gpsData.time += (float) rawTime[9] * 0.01;
    gpsData.time += (float) rawTime[10] * 0.001;

    //calculate latitude
    gpsData.latitude = rawLatitude[3]*10.0;
    gpsData.latitude += rawLatitude[4]*1.0;
    gpsData.latitude += rawLatitude[6]*0.1;
    gpsData.latitude += rawLatitude[7]*0.01;
    gpsData.latitude += rawLatitude[8]*0.001;
    gpsData.latitude += rawLatitude[9]*0.0001;
    gpsData.latitude /= 60;  //Converts from dd.mmmmmm to decimal degrees. (60 minutes in a degree)
    //Then add the degrees (ranges from -90 to +90)
    gpsData.latitude += rawLatitude[1]*10.0;
    gpsData.latitude += rawLatitude[2]*1.0;

    if (latitudeNS == 'S'){
        gpsData.latitude *= -1;
    }


    //calculate longitude
    gpsData.longitude = rawLongitude[4]*10.0;
    gpsData.longitude += rawLongitude[5]*1.0;
    gpsData.longitude += rawLongitude[7]*0.1;
    gpsData.longitude += rawLongitude[8]*0.01;
    gpsData.longitude += rawLongitude[9]*0.001;
    gpsData.longitude += rawLongitude[10]*0.0001;
    gpsData.longitude /= 60;  //Converts from ddd.mmmmmm to decimal degrees. (60 minutes in a degree)
    //Then add the degrees (ranges from -180 to +180)
    gpsData.longitude += rawLongitude[1]*100.0;
    gpsData.longitude += rawLongitude[2]*10.0;
    gpsData.longitude += rawLongitude[3]*1.0;

    if (longitudeEW == 'W'){
        gpsData.longitude *= -1;
    }

    //calculate satellites
    if (rawSatellites[2] == 10) gpsData.satellites = rawSatellites[1];
    else gpsData.satellites = rawSatellites[1]*10 + rawSatellites[2];

    //calculate altitude - tricky because of unknown 1-3 digits preceeding the decimal
    int i = 1;
    long int multiplier = 10;
    int decimalPoint = 0;
    gpsData.altitude = 0;
    float tAltitude = 0;
    for (i = 1; i < 8; i++) //this code first generates an 6 digit decimal number
    {
        if (rawAltitude[i] == 0x10) //check for decimal point
        {
            decimalPoint = i;
        } else {
            tAltitude += (float) (rawAltitude[i]*1000000 / multiplier);
            multiplier *= 10;
        }
    }
    decimalPoint = decimalPoint - 2;
    multiplier = 100000;
    while (decimalPoint > 0) //then divides it according to the placement of the decimal
    {
        multiplier = multiplier / 10;
        decimalPoint--;
    }
    gpsData.altitude = (int)(tAltitude / multiplier);

    //calculate heading - tricky because of unknown 1-3 digits preceeding the decimal
    i = 1;
    multiplier = 10;
    decimalPoint = 0;
    gpsData.heading = 0;
    float tHeading = 0;
    for (i = 1; i < 6; i++) //this code first generates an 5 digit decimal number
    {
        if (rawHeading[i] == 0x10)//check for decimal point
        {
            decimalPoint = i;
        } else {
            tHeading += (float) (rawHeading[i]*100000 / multiplier);
            multiplier *= 10;
        }
    }
    decimalPoint = decimalPoint - 2;
    multiplier = 10000;
    while (decimalPoint > 0) //then divdes it according to the placement of the decimal
    {
        multiplier = multiplier / 10;
        decimalPoint--;
    }
    gpsData.heading = (int)(tHeading / multiplier);

    //	//calculate speed - tricky because of unknown 1-3 digits preceeding the decimal
    i = 1;
    multiplier = 10;
    decimalPoint = 0;
    gpsData.speed = 0;
    for (i = 1; i < 7; i++) //this code first generates an 6 digit decimal number
    {
        if (rawGroundSpeed[i] == 0x10)//check for decimal point
        {
            decimalPoint = i;
        } else {
            gpsData.speed += (float) (rawGroundSpeed[i]*1000000 / multiplier);
            multiplier = multiplier * 10;
        }
    }
    decimalPoint = decimalPoint - 2;
    multiplier = 100000;
    while (decimalPoint > 0) //then divdes it according to the placement of the decimal
    {
        multiplier = multiplier / 10;
        decimalPoint--;
    }
    gpsData.speed = gpsData.speed / multiplier;

    gpsData.positionFix = positionFix;

    //	while(U2STAbits.UTXBF == 1){;}
    //	U2TXREG = 44;
    //	printLongInt(gpsData.latitude);
    //	while(U2STAbits.UTXBF == 1){;}
    //	U2TXREG = 44;
    //	printLongInt(gpsData.longitude);
    //	while(U2STAbits.UTXBF == 1){;}
    //	U2TXREG = 44;
    //	printInt(gpsData.satellites);
    //	while(U2STAbits.UTXBF == 1){;}
    //	U2TXREG = 44;
    //	printLongInt(gpsData.altitude);
    //	while(U2STAbits.UTXBF == 1){;}
    //	U2TXREG = 44;
    //	printLongInt(gpsData.heading);
    //	while(U2STAbits.UTXBF == 1){;}
    //	U2TXREG = 44;
    //	printLongInt(gpsData.speed);
    //	while(U2STAbits.UTXBF == 1){;}
    //	U2TXREG = 13;

    //set arrays to initial state
    rawSatellites[1] = 0;
    rawSatellites[2] = 10;
    for (i = 0; i < 7; i++) {
        rawHeading[i] = 0;
    }
    for (i = 0; i < 8; i++) {
        rawAltitude[i] = 0;
        rawGroundSpeed[i] = 0;
    }



}

void transmitData(void) {
    int i;
    char *dataGPS = &gpsData;
    char spiChecksum = 0;


    PORTBbits.RB15 = 0; //slave enabled
    for (i = 0; i < sizeof(struct GPSData); i++)
    {

        while (SPI1STATbits.SPITBF != 0) {;}
        SPI1BUF = dataGPS[0];
//        spiChecksum ^= dataGPS[0];
        dataGPS++;
    }
//    while (SPI1STATbits.SPITBF != 0) {;}
//    SPI1BUF = spiChecksum;
    //Delay_half_us(110);
    PORTBbits.RB15 = 1; //slave disabled

}


//***********************************************************************************************************************

char CharConvert(unsigned int checkSumHalf) {
    char charOut = 0;

    if (checkSumHalf >= 0 && checkSumHalf <= 9){
        charOut = checkSumHalf + 0x30;
    }
    else if (checkSumHalf >= 0xA && checkSumHalf <= 0xF){
        charOut = checkSumHalf + 0x37;
    }
    return charOut;
}

char HexConvert(unsigned char asciiSymbol) {
    char hexOut = 0;
    if (asciiSymbol == 0x2E)
        hexOut = 0x10;
    else if (asciiSymbol >= 0x30 && asciiSymbol <= 0x39){
        hexOut = asciiSymbol - 0x30;
    }
    else if (asciiSymbol >= 0x41 && asciiSymbol <= 0x46){
        hexOut = asciiSymbol - 0x37; //Letter "F"(ASCII 0x46) becomes 0xF
    }
    return hexOut;
}

int Delay_ms(unsigned int millisec) {
    unsigned int i;
    unsigned int j = 0;
    for (i = 0; i <= millisec; i++) {
        for (j = 0; j <= 2750; j++) {
            asm("nop");
        }
    }
    return 0;
}

int Delay_half_us(unsigned int microsec) {
    unsigned int i;
    for (i = 0; i <= microsec; i++) {
        //for (j=0; j<=1; j++)
        //	{
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        //}
    }
    return 0;
}


uint32_t start_time = 0;
bool toggle = true;
int main(void) {
    initTimer4();

    delay(200); //wait a bit to make sure we're getting consistent power
    
    initLogger();
    initLED();
     debug("Letting GPS module start up for 2 sec..");
     setLED(1);
     while(1);
    delay(2000);
    
    initSPI(); //initialize SPI bus
    
   
    while(1);
//
//    while(1){
//        if (time_ms - start_time > 1000){
//            start_time = time_ms;
//            setLED(toggle);
//            toggle = !toggle;
//            debug("hello how are you today!");
//        }
//    }

    initUART(1, 9600);
    while(1){
        if (U1STAbits.URXDA){
            U2TXREG = U1RXREG;
        }
    }
    configGPS(); //configure gps if battery was removed
    
    while (1) {
        if (newData){
            debug("receiv new data");
            if (readGPSData()) {
                convertData();
            }
            transmitData();	//to SPI
            newData = 0;
        }

    }
}






