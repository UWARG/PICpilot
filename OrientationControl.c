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
float kd_gain[5] = {3.80577445030212, 29.2125988006591, 0, 0, 0};//{25.9,39.8,0};//{25.9, 39.8, -8.38}; //14.619,35.7086
float kp_gain[5] = {1, 1, 1, 1, 1};
float ki_gain[5]= {0, 0, 0.5, 0, 0};
//Interal Values
float sum_gain[5] = {0, 0, 0, 0, 0};
float lastControlTime[5] = {0, 0, 0, 0, 0};
//Derivative Values
float lastError[5] = {0, 0, 0, 0, 0}; //[0],[1],[2] are currently unused

//TODO: Delete all code related to angle_bias
float angle_zero[5];
char integralFreeze = 0;

float controlSignalAltitude(float setpoint, float output, float time){
    //Integral Calculations
    float dTime = time - lastControlTime[ALTITUDE];
    lastControlTime[ALTITUDE] = time;
    //To ensure that the time is valid and doesn't suddenly spike.
    if (dTime > 1 || dTime <= 0){
        dTime = 1;
    }

    float error = setpoint - output;
    if (integralFreeze == 0){
        sum_gain[ALTITUDE] += (error * dTime);
    }

    float controlSignal = (error * kp_gain[ALTITUDE] + (sum_gain[ALTITUDE] * ki_gain[ALTITUDE]));
    return controlSignal;
}



float controlSignalHeading(float setpoint, float output, float time) { // function to find output based on gyro acceleration and PWM input

    //Take into account Heading overflow (330 degrees and 30 degrees is a 60 degree difference)
    if (setpoint + 180 < output){
        setpoint += 360;
    }
    
    //Integral Calculations
    float dTime = time - lastControlTime[HEADING];
    lastControlTime[HEADING] = time;
    //To ensure that the time is valid and doesn't suddenly spike.
    if (dTime > 1 || dTime <= 0){
        dTime = 1;
    }

    float error = setpoint - output;
    if (integralFreeze == 0){
        sum_gain[HEADING] += (error * dTime);
    }

    float controlSignal = ALTITUDE_PITCH_SCALE_FACTOR * (error * kp_gain[HEADING] + (sum_gain[HEADING] * ki_gain[HEADING]));
    return controlSignal;
}
int controlSignalAngles(float setpoint, float output, unsigned char type, float SERVO_SCALE_FACTOR_ANGLES, float time) { // function to find output based on gyro acceleration and PWM input
    float dTime = time - lastControlTime[type];
    lastControlTime[type] = time;
    //To ensure that the time is valid and doesn't suddenly spike.
    if (dTime > 1){
        dTime = 0;
    }

    float error = setpoint - output;
    if (integralFreeze == 0){
        sum_gain[type] += (error * dTime);
    }

    int controlSignal = SERVO_SCALE_FACTOR_ANGLES * (error * kp_gain[type] + (sum_gain[type] * ki_gain[type]));
    return controlSignal;
}
int controlSignal(float setpoint, float output, unsigned char type) { // function to find output based on gyro acceleration and PWM input
    int controlSignal = SERVO_SCALE_FACTOR * (setpoint - output * kd_gain[type]) + MIDDLE_PWM;
    return controlSignal;
}
void getAngleBias(){
   VN100_SPI_GetYPR(0, &angle_zero[YAW], &angle_zero[PITCH], &angle_zero[ROLL]);
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