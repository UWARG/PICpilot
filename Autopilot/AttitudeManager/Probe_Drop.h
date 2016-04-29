/*
 * File:   Probe_Drop.h
 * Author: WARG
 *
 * Created on February 17, 2016, 8:52 PM
 */

#ifndef PROBE_DROP_H
#define	PROBE_DROP_H

#ifdef	__cplusplus
extern "C" {
#endif

#define PROBE1 1
#define PROBE2 2
#define PROBE3 3

#define MAX_PROBE 3

#define PROBE1_DEFAULT 105
#define PROBE2_DEFAULT -150
#define PROBE3_DEFAULT 0

#define PROBE1_ACTIVE -240
#define PROBE2_ACTIVE -370
#define PROBE3_ACTIVE -370

void dropProbe(char num);
void resetProbe(char num);
int probePWM(char num);
char getProbeStatus();

#ifdef	__cplusplus
}
#endif

#endif	/* PROBE_DROP_H */

