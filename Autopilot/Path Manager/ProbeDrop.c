#include <stdio.h>
#include <math.h>

#include "ProbeDrop.h"
#include "PathManager.h"
#include "Dubins.h"

typedef char bool;
#define true 1
#define false 0

double GRAVITY = 9.81;

void getVelocityOfWind (float* groundVelocity, float* windVelocity, float& velocityOfWind) {
    for (int i = 0; i < 3; i++) {
        velocityOfWind[i] = sqrt(windVelocity[i] - groundVelocity[i]);
    }
}

bool probeDrop(bool *verifiedDrop, Vector *targetPosition, Vector *currentPosition, float *altitude, float* groundVelocity, float* windVelocity) {
    if (!verifiedDrop) {
        return false;
    }
    double DRAG_CONSTANT;
    double SURFACE_AREA;
    double DENSITY;
    double MASS;

    double terminalVelocity = sqrt((2*MASS*GRAVITY)/(DRAG_CONSTANT*AREA));
    double characteristicTime = terminalVelocity/GRAVITY;
    
    //The time until impact when the probe is dropped
    double impactTime = characteristicTime*acosh(pow(M_E,altitude/(terminalVelocity*characteristicTime)));
    
    //The horizontal distance from the target
    double distanceFromTarget = sqrt(pow(currentPosition->x - targetPosition->x,2) + pow(currentPosition->y - targetPosition->y,2));
    
    
    //double totalDropTime = sqrt(2*altitude/GRAVITY);
    
    //The time it will take the plane to reach the target
    double timeUntilArrival = distanceFromTarget/windVelocity;
    
    return timeUntilArrival <= impactTime;
    
    
}