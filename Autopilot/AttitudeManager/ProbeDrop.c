/*
 * @author Jack Paduchowski
 * @file ProbeDrop.c
 * @created September 8, 2017
 * This file was created for the embedded software bootcamp
 */


#include "PWM.h"

/*
 * MIN_PWM = -1024 (From PWM.h) and will put servo in closed position
 * MAX_PWM = 1024 and will put servo in open position
 * Channels 5, 6, and 7 will be used for servos 1, 2, and 3 respectively
 */

void servoStartup(){ //run this at startup to ensure servos are all closed
    setPWM(5, MIN_PWM);
    setPWM(6, MIN_PWM);
    setPWM(7, MIN_PWM);
}

void probeDrop(int probeNum, int openState){ //probeNum tells function which probe to drop, openState will set servo to open if int = 1 and closed if int = 0
    if(probeNum == 1){
        if(openState == 1){
            setPWM(5, MAX_PWM);
        } else{
            setPWM(5, MIN_PWM);
        }
    } else if(probeNum == 2){
        if(openState == 1){
            setPWM(6, MAX_PWM);
        } else{
            setPWM(6, MIN_PWM);
        }
    }else{
        if(openState == 1){
            setPWM(7, MAX_PWM);
        } else{
            setPWM(7, MIN_PWM);
        }
    }
    
}
