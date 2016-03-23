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

int prob_drop[2] = {-235, -235};
// prob_drop_old(channle 6) close -240  open 500
// prob_drop_new(channle 7) close -230  open 510

//char dropping = 0;
//int prob_old_state = 0
void dropProbe(char num)
{
    prob_drop[num - 1] = 510;
}


void resetProbe(char num)
{
    prob_drop[num -1] = -235;
}

int probeStatus(char num)
{
    return prob_drop[num - 1];
}
//char getProbeDropState(){
