/*
 * File:   InputCapture.h
 * Author: Chris Hajduk
 *
 * Created on March 4, 2013, 10:31 PM
 */

extern int icTimeDiff[];
extern int t1[9];
extern int t2[9];

extern short checkic[9];

//function prototypes
void delay (void);
void initInputCapture(void);
//void initTimer(void);
void initIC(void);
void init_t2(void);
void init_EasyVarNames();