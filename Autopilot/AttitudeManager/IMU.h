/**
 * @file IMU.h
 * @author Ian Frosst
 * @date March 16, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#ifndef IMU_H
#define	IMU_H

#define VN100 0
#define MPU9250 1
#define MPU6050 2

#define USE_IMU VN100

typedef struct IMU_data {
    float roll, pitch, heading;
    float rollRate, pitchRate, yawRate;
} IMU_data;


void initIMU();

void updateIMU();

#endif	/* IMU_H */

