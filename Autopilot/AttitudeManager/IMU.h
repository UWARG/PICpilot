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

typedef struct IMUData {
    float roll, pitch, heading;
    float rollRate, pitchRate, yawRate;
} IMUData;

/**
 * Initializes the IMU. This also must initialize the interface for the IMU (SPI, I2C, etc.)
 */
void initIMU(void);

/**
 * Retrieves new orientation data from the IMU.
 */
void updateIMU(void);

#endif

