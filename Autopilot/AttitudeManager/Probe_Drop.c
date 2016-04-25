/*
 * File:   StateMachine.c
 * Author: Jim Lin
 *
 * Created on February 9, 2016, 9:00 PM
 *
 * range from -1024 ->1024
 *
 * Rev_A:
 * PWM calibration: Eric Field
 * Probe Drop v3 mechanical assembly: Eric Field
 * PWM Calibration on April 25/2016
 *
 * Rev_A Notes:
 * PWM calibration for Corona CS-939MG metal geared servos
 * Mechanical v3 - implemented geometry changes to improve release efficency
 * Implemented separate PWM values for reset_prob function for each servo
 * determined through calibration
 */

#include "StateMachine.h"
#include "../Common/debug.h"
#include "Probe_Drop.h"

static int prob_drop[3] = {175,-50,75}; // same as corresponding resetProbe values
// prob_drop_1 (channel 6)
// prob_drop_2 (channel 7)
// prob_drop_3 (channel 8)

//char dropping = 0;
//int prob_old_state = 0
void dropProbe(char num)
{
    prob_drop[num - 1] = -350; // from calibration
}

void resetProbe(char num) // close on probe
{
    if (num == 1)
    {
        prob_drop[num - 1] = 175; // from calibration
    }
    else if (num == 2)
    {
        prob_drop[num - 1] = -50; // from calibration
    }
    else
        prob_drop[num - 1] = 75; // from calibration
}

int probeStatus(char num) // open releasing probe
{
    return prob_drop[num - 1];
}
//char getProbeDropState(){
