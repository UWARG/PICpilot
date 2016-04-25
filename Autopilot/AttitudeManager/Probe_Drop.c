/*
 * File:   StateMachine.c
 * Author: Jim Lin
 *
 * Created on February 9, 2016, 9:00 PM
 * range from -1024 ->1024
 *
 */

#include "StateMachine.h"
#include "../Common/debug.h"
#include "Probe_Drop.h"

static int prob_drop[3] = {175,-50,0};
// prob_drop_1 (channel 6)
// prob_drop_2 (channel 7)

//char dropping = 0;
//int prob_old_state = 0
void dropProbe(char num)
{
    prob_drop[num - 1] = -370;
}


void resetProbe(char num)
{
    if (num==1)
    {
        prob_drop[num -1] = 175;
    }
    else if (num==2)
    {
        prob_drop[num- 1] = -50;
    }
    else
        prob_drop[num-1] = 0;
}

int probeStatus(char num)
{
    return prob_drop[num - 1];
}
//char getProbeDropState(){
