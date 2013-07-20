/*
 * File:   InputCapture.h
 * Author: Chris Hajduk
 *
 * Created on March 4, 2013, 10:31 PM
 */

extern int icTimeDiff[];
extern int t1[8];
extern int t2[8];

extern short checkic[8];

//function prototypes
void delay (void);
void initInputCapture(char initIC);
//void initTimer(void);
void initIC(char initIC);
void init_t2(void);
void init_EasyVarNames();
int* getICValues();