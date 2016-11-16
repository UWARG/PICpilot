#include "Drop_and_Shoot.h"

static int prob_drop[3] = {PROBE1_DEFAULT, PROBE2_DEFAULT, PROBE3_DEFAULT};
// prob_drop_1 (channel 6)
// prob_drop_2 (channel 7)
// prob_drop_3 (channel 8)

char dropped = 0;
//int prob_old_state = 0

void Drop_and_Shoot(char num){
    if (num == 1){
        dropProbe(1);
        triggerCamera();
    }
    else if (num == 2){
        dropProbe(2);
        triggerCamera();
    }
    else{
        dropProbe(3);
        triggerCamera();
    }
    dropped |= (1 << (num - 1));
}
