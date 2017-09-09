/**
* @file ProbeDrop.h
* @author JackPaduchowski
* @created September 9, 2017
*/

#ifndef ProbeDrop_H
#define ProbeDrop_H

void servoStartup(); //ensures all servos are closed on startup

void probeDrop(int probeNum, int openState); //probeNum selects which servo to use, openState sets state ( open = 1, closed = 0)

#endif //ProbeDrop_H