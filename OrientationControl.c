/*
 * File:   OrientationControl.c
 * Author: Chris Hajduk
 *
 * Created on October 29, 2013, 9:41 PM
 */
#include "OutputCompare.h"
#include "OrientationControl.h"
#include "main.h"


float kd_gyro[3] = {0, 0, 0};
float kp_accel[3] = {1, 1, 1};
float ki_accel[3]= {0.01, 0.01, 0.01};
float sum_accel[3] = {0, 0, 0};

// TODO: add integrator reset function
float SERVO_SCALE_FACTOR = -(UPPER_PWM - MIDDLE_PWM) / 45;
float angle_zero[3];
char integralFreeze = 0;

//float Angle_Bias[3];
int controlSignalAngles(float setpoint, float output, char type, float SERVO_SCALE_FACTOR_ANGLES) { // function to find output based on gyro acceleration and PWM input
    if (integralFreeze == 0){
        sum_accel[type] += (setpoint - output);
    }
    int control = SERVO_SCALE_FACTOR_ANGLES * ((setpoint - output) * kp_accel[type] + (sum_accel[type]) * ki_accel[type]);
    return control;
}
int controlSignal(float setpoint, float output, char type) { // function to find output based on gyro acceleration and PWM input
    int control = SERVO_SCALE_FACTOR * (setpoint - output * kd_gyro[type]) + MIDDLE_PWM;
    return control;
}
int getAngleBias(){
   VN100_SPI_GetYPR(0, &angle_zero[YAW], &angle_zero[PITCH], &angle_zero[ROLL]);
    }

void freezeIntegral() {
    integralFreeze = 1;
}

void unfreezeIntegral() {
    integralFreeze = 0;
}

void setIntegralSum(char YPR, float value) {
    sum_accel[YPR] = value;
}
float getIntegralSum(char YPR){
    return sum_accel[YPR];
}

float getGain(char YPR, char type){
    if (type == GAIN_KD){
        return kd_gyro[YPR];
    }
    else if (type == GAIN_KP){
         return kp_accel[YPR];
    }
    else if (type == GAIN_KI){
        return ki_accel[YPR];
    }
    else
        return -1;
}
void setGain(char YPR, char type, float value){
    if (type == GAIN_KD){
        kd_gyro[YPR] = value;
    }
    else if (type == GAIN_KP){
         kp_accel[YPR] = value;
    }
    else if (type == GAIN_KI){
        ki_accel[YPR] = value;
    }

}