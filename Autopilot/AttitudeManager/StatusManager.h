/* 
 * @file StatusManager.h
 * @author Serj Babayan
 * @created April 28, 2017, 4:23 AM
 * Manages kill mode and peripheral states of the autopilot
 */

#ifndef STATUSMANAGER_H
#define	STATUSMANAGER_H

#include <stdbool.h>

/**
 * Time elapsed in ms before we consider the heartbeat status in a 'warn' state
 */
#define HEARTBEAT_WARN_TIMEOUT 10000

/**
 * Time elapsed in ms before we consider the heratbeat status expired, and thus
 * datalink connection lost
 */
#define HEARTBEAT_EXPIRED_TIMEOUT 20000

/**
 * Timeout in ms before we consider the uhf in a warn state
 */
#define UHF_WARN_TIMEOUT 5000

/**
 * Time in ms before we consider the uhf in a expired, and thus completely
 * disconnected state (should cause aircraft to kill)
 */
#define UHF_EXPIRED_TIMEOUT 10000

typedef const enum {
    CONNECTION_OK,
    CONNECTION_WARN,
    CONNECTION_EXPIRED
} ConnectionStatus;

/**
 * Whether the heartbeat timer is still valid and we are still receiving datalink,
 * or we need to start worrying
 * @return
 */
ConnectionStatus getHeartbeatStatus(void);

/**
 * Resets the heartbeat timer to the current time. This must be called for
 * isHeartbeatTimerExpired to outputting anything other than false. It must
 * only be called when new data is received
 */
void resetHeartbeatTimer(void);

/**
 * Checks the current UHF status. If the uhf is connected, automatically resets the
 * timer
 */
void checkUHFStatus(void);

/**
 * Whether the uhf status is still valid and is considered connected. If this is
 * expired, the plane should most likely killitself
 * @return
 */
ConnectionStatus getUHFStatus(void);

#endif

