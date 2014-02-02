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
float kd_gain[4] = {0, 0, 0, 0};//{25.9,39.8,0};//{25.9, 39.8, -8.38};
float kp_gain[4] = {1, 1, 1, 1};
float ki_gain[4]= {0, 0, 0, 0};
//Interal Values
float sum_gain[4] = {0, 0, 0, 0};
float lastControlTime[4] = {0, 0, 0, 0};
//Derivative Values
float lastError[4] = {0, 0, 0, 0}; //[0],[1],[2] are currently unused

//TODO: Delete all code related to angle_bias
float angle_zero[3];
char integralFreeze = 0;

//TODO: Add derivative control to the heading
float controlSignalHeading(float setpoint, float output, float time) { // function to find output based on gyro acceleration and PWM input

    //Take into account Heading overflow (330 degrees and 30 degrees is a 60 degree difference)
    if (setpoint + 180 < output){
        setpoint += 360;
    }
    
    //Integral Calculations
    float dTime = time - lastControlTime[HEADING];
    lastControlTime[HEADING] = time;
    
    //To ensure that the time is valid and doesn't suddenly spike.
    if (dTime > 1){
        return 0;
    }
    if (integralFreeze == 0){
        sum_gain[HEADING] += (setpoint - output);
    }

    //Derivative Calculations
    float dValue = (setpoint - output) - lastError[HEADING];
    lastError[HEADING] = setpoint - output;

    float control = (dValue/dTime * kd_gain[HEADING] + (setpoint - output) * kp_gain[HEADING] + (sum_gain[HEADING] * ki_gain[HEADING] * dTime));
    return control;
}
int controlSignalAngles(float setpoint, float output, unsigned char type, float SERVO_SCALE_FACTOR_ANGLES, float time) { // function to find output based on gyro acceleration and PWM input
    float dTime = time - lastControlTime[type];
    lastControlTime[type] = time;

    //To ensure that the time is valid and doesn't suddenly spike.
    if (dTime > 1){
        return 0;
    }
    if (integralFreeze == 0){
        sum_gain[type] += (setpoint - output);
    }
    int control = SERVO_SCALE_FACTOR_ANGLES * ((setpoint - output) * kp_gain[type] + (sum_gain[type] * ki_gain[type] * dTime));
    return control;
}
int controlSignal(float setpoint, float output, unsigned char type) { // function to find output based on gyro acceleration and PWM input
    int control = SERVO_SCALE_FACTOR * (setpoint - output * kd_gain[type]) + MIDDLE_PWM;
    return control;
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
//void degreeConvert(float *mag){
//    int i = 0;
//    for (i =0; i < 3; i++){
//        mag[i] = mag[i]/constmax * 90;
//    }
//}