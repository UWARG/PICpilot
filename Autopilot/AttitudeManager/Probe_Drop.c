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

int prob_drop[3] = {-10,-10,-10};
// prob_drop_1 (channle 6) close 10  open -350
// prob_drop_2 (channle 7) close 10  open -350

//char dropping = 0;
//int prob_old_state = 0
void dropProbe(char num)
{
    prob_drop[num - 1] = -370;
}


void resetProbe(char num)
{
    prob_drop[num -1] = -10;
}

int probeStatus(char num)
{
    return prob_drop[num - 1];
}
//char getProbeDropState(){
