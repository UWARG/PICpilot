#include <math.h>
#include "ProbeDrop.h"
#include "PathManager.h"
#include "Dubins.h"

double GRAVITY = 9.81;
double DRAG_CONSTANT = 1;
double SURFACE_AREA = 1;
double DENSITY = 1;
double MASS = 1;

void getVelocityOfWind(float* groundVelocity, float* windVelocity, float* velocityOfWind){
    int i = 0;
    for (i = 0; i < 3; i++){
        velocityOfWind[i] = sqrt(windVelocity[i] - groundVelocity[i]);
    }
}

char probeDrop(char verifiedDrop, Vector* targetPosition, float* currentPosition, float* altitude, float* groundVelocity, float* windVelocity){
    if (!verifiedDrop) {
        return 0;
    }

    double terminalVelocity = sqrt((2*MASS*GRAVITY)/(DRAG_CONSTANT*SURFACE_AREA));
    double characteristicTime = terminalVelocity/GRAVITY;
    
    //The time until impact when the probe is dropped
    double impactTime = characteristicTime*acos(pow(MASS,(*altitude)/(terminalVelocity*characteristicTime))); //TODO: is acos correct here?
    
    //The horizontal distance from the target
    double distanceFromTarget = sqrt(pow(currentPosition[0] - targetPosition->x,2) + pow(currentPosition[1] - targetPosition->y,2));
    
    
    //double totalDropTime = sqrt(2*altitude/GRAVITY);
    
    //The time it will take the plane to reach the target
    double timeUntilArrival = distanceFromTarget/(*windVelocity);
    
    return timeUntilArrival <= impactTime;
    
    
}