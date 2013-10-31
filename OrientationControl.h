/* 
 * File:   OrientationControl.h
 * Author: Chris Hajduk
 *
 * Created on October 29, 2013, 10:46 PM
 */

//global variables

float extern angle_zero[3];




//Function Prototypes
int controlSignalAngles(float setpoint, float output, char type, float SERVO_SCALE_FACTOR_ANGLES);
int controlSignal(float setpoint, float output, char type);
int getAngleBias();
void freezeIntegral();
void unfreezeIntegral();

void resetIntegral(char type);

