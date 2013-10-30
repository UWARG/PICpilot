/*
 * File:   OrientationControl.c
 * Author: Chris Hajduk
 *
 * Created on October 29, 2013, 9:41 PM
 */

/*
 * Values
 */

// TODO: Move all orientation control code in here
// TODO: add VectorNav positioning bias reset
// TODO: add integrator freeze function
// TODO: add integrator reset function

float SERVO_SCALE_FACTOR = -(UPPER_PWM - MIDDLE_PWM) / 45;

//float Angle_Bias[3];
int controlSignalAngles(float setpoint, float output, float gain, float integralGain, float* integralSum, float SERVO_SCALE_FACTOR_ANGLES) { // function to find output based on gyro acceleration and PWM input
    (*integralSum) += (setpoint - output);

    int control = SERVO_SCALE_FACTOR_ANGLES * ((setpoint - output) * gain + (*integralSum) * integralGain);
    return control;
}
int controlSignal(float setpoint, float output, float gain) { // function to find output based on gyro acceleration and PWM input
    int control = SERVO_SCALE_FACTOR * (setpoint - output * gain) + MIDDLE_PWM;
    return control;
}

//int getAngleBias(){
//    VN100_SPI_GetYPR(0, &Angle_Bias[YAW], &Angle_Bias[PITCH], &Angle_Bias[ROLL]);
//    }