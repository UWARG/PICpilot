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

static int prob_drop[3] = {PROBE1_DEFAULT, PROBE2_DEFAULT, PROBE3_DEFAULT};
// prob_drop_1 (channel 6)
// prob_drop_2 (channel 7)

char dropped = 0;
//int prob_old_state = 0
void dropProbe(char num)
{
    if (num > 0){
        prob_drop[num - 1] = PROBE1_ACTIVE;
        //Mark the probe as dropped
        dropped |= (1 << (num - 1));
    }
}


void resetProbe(char num)
{
    if (num==1)
    {
        prob_drop[num -1] = PROBE1_DEFAULT;
    }
    else if (num==2)
    {
        prob_drop[num- 1] = PROBE2_DEFAULT;
    }
    else
        prob_drop[num-1] = PROBE3_DEFAULT;
    //Unmark the probe as dropped
    dropped &= ~(1 << (num - 1));
}

int probePWM(char num)
{
    return prob_drop[num - 1];
}

char getProbeStatus(){
    //Returns bit indicating which probes were dropped
    return dropped;
}
