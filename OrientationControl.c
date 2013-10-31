/*
 * File:   OrientationControl.c
 * Author: Chris Hajduk
 *
 * Created on October 29, 2013, 9:41 PM
 */
#include "OutputCompare.h"


float kd_gyro[3] = {0, 0, 0};
float kp_accel[3] = {1, 1, 1};
float ki_accel[3]= {0.01, 0.01, 0.01};
float sum_accel[3] = {0, 0, 0};

// TODO: add integrator reset function

float SERVO_SCALE_FACTOR = (-(UPPER_PWM - MIDDLE_PWM) / 45);
float angle_zero[3];
char integralfreeze = 0;

//float Angle_Bias[3];
int controlSignalAngles(float setpoint, float output, char type, float SERVO_SCALE_FACTOR_ANGLES) { // function to find output based on gyro acceleration and PWM input
    if (!integralfreeze)
    sum_accel[type] += (setpoint - output);

    int control = SERVO_SCALE_FACTOR_ANGLES * ((setpoint - output) * kp_accel[type] + (sum_accel[type]) * ki_accel[type]);
    return control;
}
int controlSignal(float setpoint, float output, char type) { // function to find output based on gyro acceleration and PWM input
    int control = SERVO_SCALE_FACTOR * (setpoint - output * kd_gyro[type]) + MIDDLE_PWM;
    return control;
}

int getAngleBias(){
   VN100_SPI_GetYPR(0, &angle_zero[0], &angle_zero[1], &angle_zero[2]);
    }

void setfreeze() {
    integralfreeze = 1;
}

void unfreeze() {
    int i = 0;
    integralfreeze = 0;
    for (i = 0; i < 3; i++) {
        sum_accel[i] = 0;
    }
}

void reset(char type) {
    sum_accel[type] = 0;
}

