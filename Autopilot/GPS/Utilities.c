/**
 * @file Utilities.h
 * @author Serge Babayan
 * @date @date April 23, 2017, 3:38AM
 * 
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

char byteToHexString(unsigned int checkSumHalf) {
    char charOut = 0;

    if (checkSumHalf >= 0 && checkSumHalf <= 9){
        charOut = checkSumHalf + 0x30;
    }
    else if (checkSumHalf >= 0xA && checkSumHalf <= 0xF){
        charOut = checkSumHalf + 0x37;
    }
    return charOut;
}

char asciiToHex(unsigned char asciiSymbol) {
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
