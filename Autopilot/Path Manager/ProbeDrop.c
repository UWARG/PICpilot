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
    double distanceFromTarget = sqrt(sq(currentPosition->x, targetPosition->x) + (currentPosition->y, targetPosition->y));
    
    double totalDropTime = sqrt(2*altitude/GRAVITY);
    
    double timeUntilRelease = distanceFromTarget/windVelocity;
    
    //An angle from the x-axis
   // double direction = atan((targetPosition->y - currentPosition->y)/(targetPosition->x - currentPosition->x));
    
    
    
    return timeUntilRelease <= totalDropTime;
    
    
}