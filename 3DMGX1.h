//Define variables for global use in the code
#define TRUE	1
#define FALSE	0

//All the 3DMGX1 commands
#define sRawData 0x01
#define sGyroVectors 0x02
#define sInstantaneousVectors 0x03
#define sInstantaneousQuaternion 0x04
#define sGyroQuaternion 0x05
#define sGyroBias 0x06
#define sTemp 0x07
#define sReadEEPROM 0x08
#define sWriteEEPROM 0x09
#define sInstantaneousMatrix 0x0A
#define sGyroMatrix 0x0B
#define sGyroQuaternionVectors 0x0C
#define sInstantaneousEulerAngles 0x0D
#define sGyroEulerAngles 0x0E
#define sTare 0x0F
#define sContinuousMode 0x10
#define sRemoveTare 0x11
#define sGyroQuaternionInstantaneousVectors 0x12
#define sWriteGains 0x24
#define sReadGains 0x25
#define sSelfTest 0x27
#define sReadEEPROMChecksum 0x28
#define sWriteEEPROMChecksum 0x29
#define sGyroEulerAnglesAccelRate 0x31
#define sInitializeHardIronFieldCalibration 0x40
#define sCollectHardIronFieldCalibrationData 0x41
#define sComputeHardIronFieldCalibrationData 0x42
#define sFirmwareVersion 0xF0
#define sDeviceSerialNumber 0xF1

//external variables
struct IMU
{
    double roll;		/* 32 bit signed Euler roll angle in degrees */
    double pitch;		/* 32 bit signed Euler pitch angle in degrees */
    double yaw;                 /* 32 bit signed Euler yaw angle in degrees */
    short accel_x;		/* 16 bit signed acceleration along the X axis */
    short accel_y;		/* 16 bit signed acceleration along the Y axis */
    short accel_z;		/* 16 bit signed acceleration along the Z axis */
    short rate_x;		/* 16 bit signed rotation rate around X axis */
    short rate_y;		/* 16 bit signed rotation rate around Y axis */
    short rate_z;		/* 16 bit signed rotation rate around Z axis */
    short ticks;		/* 16 bit 3DM-GX1 internal time stamp */
    short chksum;		/* checksum */
    unsigned char accelGainScale;/* 8 bit unsigend gain factor for acceleration stored at EEPROM#230*/
    unsigned char gyroGainScale;/* 8 bit unsigned gain factor for gyroscopes stored at EEPROM#130*/
};

// The receieving message size for each command above, excluding the serial and firmware numbers, otherwise it is 5 bytes
extern const int messageSize[];
// List all function prototypes below:
double* calcEulerAngles();
double* calcAccelRate();
int checkValidity(unsigned char* data);
struct IMU getCurrentData();
void updateCurrentData(char* command, int dataLength);
void init_3DMGX1();
unsigned char* returnCommandData();

