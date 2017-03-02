/* 
 * File:   OrientationControl.h
 * Author: Ian Frosst
 *
 * Created on March 2, 2017, 3:29 PM
 */

#ifndef ORIENTATIONCONTROL_H
#define	ORIENTATIONCONTROL_H

#define GAIN_KD 0
#define GAIN_KP 1
#define GAIN_KI 2

#define PID_CHANNELS 6 // to start

#define PID_ROLL_RATE   0
#define PID_PITCH_RATE  1
#define PID_YAW_RATE    2
#define PID_ROLL_ANGLE  3
#define PID_PITCH_ANGLE 4
#define PID_HEADING     5

#define PID_RESET_TIME 1000 // timeout to reset I and D terms (ms)

typedef struct PID_val {
    // Gains
    float Kp;
    float Ki;
    float Kd;
    
    unsigned long lastTime; // for derivative control
    float lastErr;  
    float integral;
    unsigned long imax; // maximum value for integral
} PID_val;

#endif	/* ORIENTATIONCONTROL_H */

