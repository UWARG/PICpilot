/*
 * @file StatusManager.c
 * @author Serj Babayan
 * @created April 28, 2017, 4:23 AM
 */

#include "StatusManager.h"
#include "../Common/Clock/Timer.h"
#include "PWM.h"
#include "Network/Datalink.h"
#include <stdbool.h>

static uint32_t heartbeat_timer;
static uint32_t uhf_timer;

ConnectionStatus getHeartbeatStatus() {
    if (getTime() - heartbeat_timer > HEARTBEAT_EXPIRED_TIMEOUT) {
        return CONNECTION_EXPIRED;
    } else if (getTime() - heartbeat_timer > HEARTBEAT_WARN_TIMEOUT) {
        return CONNECTION_WARN;
    }
    return CONNECTION_OK;
}

void resetHeartbeatTimer() {
    heartbeat_timer = getTime();
}

void checkUHFStatus() {
    if (getPWMInputStatus() == PWM_STATUS_UHF_LOST) {
        if (uhf_timer == 0) { //0 indicates that this is the first time we lost UHF
            uhf_timer = getTime();
        }
    } else {
        uhf_timer = 0; //set it back to 0 otherwise to reset state
    }
}

ConnectionStatus getUHFStatus() {
    if (uhf_timer == 0) {
        return CONNECTION_OK;
    }

    if (getTime() - uhf_timer > UHF_EXPIRED_TIMEOUT) {
        return CONNECTION_EXPIRED;
    } else if (getTime() - uhf_timer > UHF_WARN_TIMEOUT) {
        return CONNECTION_WARN;
    }

    return CONNECTION_OK;
}
