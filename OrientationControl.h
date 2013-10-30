/* 
 * File:   OrientationControl.h
 * Author: Chris Hajduk
 *
 * Created on October 29, 2013, 10:46 PM
 */





//Function Prototypes
int controlSignalAngles(float setpoint, float output, float gain, float integralGain, float* integralSum, float SERVO_SCALE_FACTOR_ANGLES);
int controlSignal(float setpoint, float output, float gain);

