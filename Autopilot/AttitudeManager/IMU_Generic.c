/**
 * @file IMU_Generic.c
 * @author Ian Frosst
 * @date March 16, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#include "IMU.h"

IMU_data imu = {0,0,0,0,0,0};

float getRoll(){
    return imu.roll;
}
float getPitch(){
    return imu.pitch;
}
float getYaw(){
    return imu.heading;
}
float getRollRate(){
    return imu.rollRate;
}
float getPitchRate(){
    return imu.pitchRate;
}
float getYawRate(){
    return imu.yawRate;
}