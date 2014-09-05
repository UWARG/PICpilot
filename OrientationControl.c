/*
 * File:   OrientationControl.c
 * Author: Chris Hajduk
 *
 * Created on October 29, 2013, 9:41 PM
 */
#include "OutputCompare.h"
#include "OrientationControl.h"
#include "main.h"
#include "AttitudeManager.h"
#include "VN100.h"

//float constmax = 2.7188;


//TODO: Change these variable names to more generic names for inclusion of heading
//25.2125988006591
float kd_gain[6] = {0, 20, 16.5748023987, 15, 20, 50};
float kp_gain[6] = {1, 0.5, 1.5, 1.5, 1.25, 0.05};
float ki_gain[6]= {0, 0, 0, 0, 0, 0};
//Interal Values
float sum_gain[6] = {0, 0, 0, 0, 0, 0};
long int lastControlTime[6] = {0, 0, 0, 0, 0, 0};
//Derivative Values
int lastError[6] = {0, 0, 0, 0, 0, 0}; //[0],[1],[2] are currently unused

char integralFreeze = 0;


int controlSignalThrottle(int setpoint, int output){
    int error = setpoint - output;
    if (integralFreeze == 0){
        sum_gain[THROTTLE] += (float)error;
    }
    int controlSignal = (int)(THROTTLE_SCALE_FACTOR * (error * kp_gain[THROTTLE] + sum_gain[THROTTLE] * ki_gain[THROTTLE]));
    return controlSignal;
}

int controlSignalAltitude(int setpoint, int output){
    int error = setpoint - output;
    if (integralFreeze == 0){
        sum_gain[ALTITUDE] += (float)error;
    }

    //Derivative Calculations ---Not necessarily needed for altitude
    int dValue = error - lastError[ALTITUDE];
    lastError[ALTITUDE] = error;


    int controlSignal = (int)(ALTITUDE_PITCH_SCALE_FACTOR * ((dValue * kd_gain[ALTITUDE]) + (error * kp_gain[ALTITUDE]) + (sum_gain[ALTITUDE] * ki_gain[ALTITUDE])));
    return controlSignal;
}

int controlSignalHeading(int setpoint, int output) { // function to find output based on gyro acceleration and PWM input
    //Take into account Heading overflow (330 degrees and 30 degrees is a 60 degree difference)
    if (setpoint - output > 180){
        output += 360;
    }
    else if (setpoint - output < -180){
        output -= 360;
    }
    
    int error = setpoint - output;
    if (integralFreeze == 0){
        sum_gain[HEADING] += (float)error;
    }
    
    //Derivative Calculations
    int dValue = error - lastError[HEADING];
    lastError[HEADING] = error;

    int controlSignal = (int)(HEADING_ROLL_SCALE_FACTOR * ((dValue * kd_gain[HEADING]) + (error * kp_gain[HEADING]) + (sum_gain[HEADING] * ki_gain[HEADING])));
    return controlSignal;
}
int controlSignalAngles(float setpoint, float output, unsigned char type, float SERVO_SCALE_FACTOR_ANGLES) { // function to find output based on gyro acceleration and PWM input

    float error = setpoint - output;
    if (integralFreeze == 0){
        sum_gain[type] += error;
    }

    int controlSignal = (int)(SERVO_SCALE_FACTOR_ANGLES * (error * kp_gain[type] + (sum_gain[type] * ki_gain[type])));
    return controlSignal;
}
int controlSignal(float setpoint, float output, unsigned char type) { // function to find output based on gyro acceleration and PWM input
    int controlSignal = (int)(SERVO_SCALE_FACTOR * (setpoint - output * kd_gain[type])) + MIDDLE_PWM;
    return controlSignal;
}

void freezeIntegral() {
    integralFreeze = 1;
}

void unfreezeIntegral() {
    integralFreeze = 0;
}

void setIntegralSum(unsigned char YPRH, float value) {
    sum_gain[YPRH] = value;
}
float getIntegralSum(unsigned char YPRH){
    return sum_gain[YPRH];
}

float getGain(unsigned char YPRH, unsigned char type){
    if (type == GAIN_KD){
        return kd_gain[YPRH];
    }
    else if (type == GAIN_KP){
         return kp_gain[YPRH];
    }
    else if (type == GAIN_KI){
        return ki_gain[YPRH];
    }
    else
        return -1;
}
void setGain(unsigned char YPRH, unsigned char type, float value){
    if (type == GAIN_KD){
        kd_gain[YPRH] = value;
    }
    else if (type == GAIN_KP){
         kp_gain[YPRH] = value;
    }
    else if (type == GAIN_KI){
        ki_gain[YPRH] = value;
    }

}