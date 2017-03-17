/**
 * @file IMU_MPU9250.c
 * @author Ian Frosst
 * @date Match 17, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

/**
 * @brief Driver for the MPU9250 IMU. 
 * This implements the generic IMU interface.
 */

#include "IMU.h"

#if USE_IMU == MPU9250

typedef enum {
    
} MPU_register;

void initIMU() {
    initSPI(IMU_SPI_PORT, 1000, SPI_MODE0, SPI_MASTER);
    
    SPI_SS(IMU_SPI_PORT, PIN_LOW);
    SPI_TX_RX(IMU_SPI_PORT, READ);
    byte resp = SPI_TX_RX(IMU_SPI_PORT, 0);
    SPI_SS(IMU_SPI_PORT, PIN_HIGH);
}



#endif