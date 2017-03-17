/**
 * @file IMU.h
 * @author Ian Frosst
 * @date March 16, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#ifndef IMU_H
#define	IMU_H

#include "ProgramStatus.h"
#include "main.h"

#define VN100 0
#define MPU9250 1
#define MPU6050 2

#define USE_IMU VN100

#define IMU_SPI_PORT 2


typedef struct IMU_data {
    float roll, pitch, heading;
    float rollRate, pitchRate, yawRate;
} IMU_data;

void initIMU();

void readIMU();

void IMU_setOrientation(float*);

void IMU_tare();



float getRoll();
float getPitch();
float getYaw();
float getRollRate();
float getPitchRate();
float getYawRate();

#endif	/* IMU_H */

