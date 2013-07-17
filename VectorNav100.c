/*
 * File:   VectorNav100.c
 * Author: Chris Hajduk
 *
 * Created on July 16, 2013, 10:48 PM
 */

/*
 * Use this file as a modular way to send and receive commands from the VectorNav for common IMU functions.
 */

/*
 * Command Format:
 * Read: 0x01
 * Write: 0x02
 * Write Settings: 0x03
 * Restore Factory Settings: 0x04
 * Tare: 0x05
 * Reset: 0x06
 * Known Magnetic Disturbance: 0x08
 * Known Acceleration Disturbance: 0x09
 * Set Gyro Bias: 0x0C
 */



unsigned char calculateChecksum(char* command, int length){
    unsigned char xor = 0;
    for(int i = 0; i < length; i++)
    xor ^= (unsigned char)command[i];
    return xor;
}
