/* 
 * File:   OrientationControl.h
 * Author: Chris Hajduk
 *
 * Created on October 29, 2013, 10:46 PM
 */

//global variables

float extern angle_zero[3];

#define GAIN_KD 0
#define GAIN_KP 1
#define GAIN_KI 2



//Function Prototypes
int controlSignalAngles(float setpoint, float output, char type, float SERVO_SCALE_FACTOR_ANGLES);
int controlSignal(float setpoint, float output, char type);
int getAngleBias();
void freezeIntegral();
void unfreezeIntegral();

void setIntegralSum(char YPR, float value);
float getIntegralSum(char YPR);
float getGain(char YPR, char type);
void setGain(char YPR, char type, float value);



