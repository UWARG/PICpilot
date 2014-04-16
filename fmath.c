/*
 * File:   fmath.c
 * Author: Chris Hajduk
 *
 * Created on April 12, 2014, 9:53 AM
 */
#include "fmath.h"

float sineTable[SINE_TABLE_SIZE];

void initTrigLookup(){
    int i = 0;
    for (i = 0; i < SINE_TABLE_SIZE; i++){
        sineTable[i] = sin(((float)i / SINE_TABLE_SIZE) * PI / 2);
    }
}

float lookup(int val){
    int divisor = (65536/SINE_TABLE_SIZE);
    int index = (val/divisor);
    float A = sineTable[index & (SINE_TABLE_SIZE - 1)];
    float B = sineTable[(index + 1) & (SINE_TABLE_SIZE - 1)];
    float slope = (val & (divisor - 1))/ (float)divisor;
    return slope * (B - A) + A;
}

float fSin(float val){
    if (val >= 0 && val < PI/2){
        return lookup(val * 65536 / (PI/2));
    }
    else if (val >= PI/2 && val < PI){
        return 1 - lookup((val - PI/2) * 65536 / (PI/2));
    }
    else if (val >= PI && val < 3 * PI / 2){
        return lookup((val - PI) * 65536 / (PI/2)) * -1;
    }
    else if (val >= 3 * PI / 2 && val < 2 * PI) {
        return (1 - lookup((val - 3 * PI / 2) * 65536 / (PI/2))) * -1;
    }
    else if (val > 2*PI){
        return fSin(val - 2 * PI);
    }
    else if (val < 0){
        return fSin(val + 2 * PI);
    }
    return 0;
}

float fCos(float val){
    return fSin(val + PI/2);
}

float fTan(float val){
    return fSin(val)/fCos(val);
}

