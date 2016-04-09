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

void dropProbe(char num);
void resetProbe(char num);
int probeStatus(char num);

#ifdef	__cplusplus
}
#endif

#endif	/* PROBE_DROP_H */

