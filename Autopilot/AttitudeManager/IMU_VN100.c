/**
 * @file IMU_VN100.c
 * @author Ian Frosst
 * @date Match 17, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

/**
 * @brief Driver for the VN-100 IMU. 
 * This abstracts the VectorNav library into a generic 
 * interface that will be used for other IMUs.
 */

#include "IMU.h"
#include "VN100.h"
#include <string.h>

#if USE_IMU == VN100

//Defined in the opposite order for rates of rotation
//Be careful not to confuse the constants
#define IMU_YAW_RATE     2
#define IMU_PITCH_RATE   1
#define IMU_ROLL_RATE    0

extern IMU_data imu;

void initIMU() {
    VN100_initSPI();
    
    char model[12];
    VN100_SPI_GetModel(0, model);
    if (strcmp(model, "VN-100T-SMD") == 0 || strcmp(model, "VN-100T") == 0){
        setSensorStatus(VECTORNAV, SENSOR_CONNECTED & TRUE);
        //IMU position matrix
        // offset = {x, y, z}
        float cal_x = -90;
        float cal_y = -90;
        float cal_z = 0.0;
        float offset[3] = {cal_x,cal_y,cal_z};

        //In order: Angular Walk, Angular Rate x 3, Magnetometer x 3, Acceleration x 3
        float filterVariance[10] = {1e-9, 1e-9, 1e-9, 1e-9, 1, 1, 1, 1e-4, 1e-4, 1e-4};
        
        IMU_setOrientation((float*)&offset);
        VN100_SPI_SetFiltMeasVar(0, (float*)&filterVariance);
        setSensorStatus(VECTORNAV, SENSOR_INITIALIZED & TRUE);
    } else {
        setSensorStatus(VECTORNAV, SENSOR_CONNECTED & FALSE);
    }
}

void readIMU() {
    float imuData[3];
    VN100_SPI_GetRates(0, (float*) &imuData);

    //TODO: This is a reminder for me to figure out a more elegant way to fix improper derivative control (based on configuration of the sensor), adding this negative is a temporary fix. Some kind of calibration command or something.
    //DO NOT ADD NEGATIVES IN THE STATEMENTS BELOW. IT IS A GOOD WAY TO ROYALLY SCREW YOURSELF OVER LATER.
    //Outputs in order: Roll,Pitch,Yaw
    imu.rollRate = imuData[IMU_ROLL_RATE];
    imu.pitchRate = imuData[IMU_PITCH_RATE];
    imu.yawRate = imuData[IMU_YAW_RATE];
    VN100_SPI_GetYPR(0, &(imu.heading), &(imu.pitch), &(imu.roll));
}

void IMU_tare() {
    VN100_SPI_Tare(0);
    VN100_SPI_WriteSettings(0);
    VN100_SPI_Reset(0);
}

void IMU_setOrientation(float* angleOffset){
    //angleOffset[0] = x, angleOffset[1] = y, angleOffset[2] = z
    angleOffset[0] = deg2rad(angleOffset[0]);
    angleOffset[1] = deg2rad(angleOffset[1]);
    angleOffset[2] = deg2rad(angleOffset[2]);

    float refRotationMatrix[9];
    refRotationMatrix[0] = cos(angleOffset[1]) * cos(angleOffset[2]);
    refRotationMatrix[1] = -cos(angleOffset[1]) * sin(angleOffset[2]);
    refRotationMatrix[2] = sin(angleOffset[1]);

    refRotationMatrix[3] = sin(angleOffset[0]) * sin(angleOffset[1]) * cos(angleOffset[2]) + sin(angleOffset[2]) * cos(angleOffset[0]);
    refRotationMatrix[4] = -sin(angleOffset[0]) * sin(angleOffset[1]) * sin(angleOffset[2]) + cos(angleOffset[2]) * cos(angleOffset[0]);
    refRotationMatrix[5] = -sin(angleOffset[0]) * cos(angleOffset[1]);

    refRotationMatrix[6] = -cos(angleOffset[0]) * sin(angleOffset[1]) * cos(angleOffset[2]) + sin(angleOffset[2]) * sin(angleOffset[0]);
    refRotationMatrix[7] = cos(angleOffset[0]) * sin(angleOffset[1]) * sin(angleOffset[2]) + cos(angleOffset[2]) * sin(angleOffset[0]);
    refRotationMatrix[8] = cos(angleOffset[0]) * cos(angleOffset[1]);
    VN100_SPI_SetRefFrameRot(0, (float*)&refRotationMatrix);
    VN100_SPI_WriteSettings(0);
    VN100_SPI_Reset(0);
}

void setAngularWalkVariance(float variance){
    float previousVariance[10];
    VN100_SPI_GetFiltMeasVar(0, (float*)&previousVariance);
    previousVariance[0] = variance;
    VN100_SPI_SetFiltMeasVar(0, (float*)&previousVariance);
    VN100_SPI_WriteSettings(0);
}

void setGyroVariance(float variance){
    float previousVariance[10];
    VN100_SPI_GetFiltMeasVar(0, (float*)&previousVariance);
    previousVariance[1] = variance; //X -Can be split up later if needed
    previousVariance[2] = variance; //Y
    previousVariance[3] = variance; //Z
    VN100_SPI_SetFiltMeasVar(0, (float*)&previousVariance);
    VN100_SPI_WriteSettings(0);
}

void setMagneticVariance(float variance){
    float previousVariance[10];
    VN100_SPI_GetFiltMeasVar(0, (float*)&previousVariance);
    previousVariance[4] = variance; //X -Can be split up later if needed
    previousVariance[5] = variance; //Y
    previousVariance[6] = variance; //Z
    VN100_SPI_SetFiltMeasVar(0, (float*)&previousVariance);
    VN100_SPI_WriteSettings(0);
}

void setAccelVariance(float variance){
    float previousVariance[10];
    VN100_SPI_GetFiltMeasVar(0, (float*)&previousVariance);
    previousVariance[7] = variance; //X -Can be split up later if needed
    previousVariance[8] = variance; //Y
    previousVariance[9] = variance; //Z
    VN100_SPI_SetFiltMeasVar(0, (float*)&previousVariance);
    VN100_SPI_WriteSettings(0);
}




/*****************************************************************************
 * Function: void adjustVNOrientationMatrix(float* adjustment);
 *
 * Preconditions: The VN100 module, and the SPI2 interface must have already been initialized.
 *
 * Overview: This function takes x,y,z positioning parameters (degrees) of the VN100 model, and applies
 * it into an orientation matrix which removes any bias.
 *
 * Input:   float* adjustment - the x, y, z rotational components of the VN100 in degrees.
 *
 * Output:  None.
 *
 *****************************************************************************/


/*****************************************************************************
 * Function: void setVNOrientationMatrix(float* angleOffset);
 *
 * Preconditions: The VN100 module, and the SPI2 interface must have already been initialized.
 *
 * Overview: This function takes a roll, pitch, and yaw angle, and sets the
 * VectorNav in a different orientation reference frame. Note that this saves the
 * values to memory, and then resets the VN100.
 *
 * Input:   The angles in an array[3], which correspond to the x,y,z components of
 * the altered reference frame.
 *
 * Output:  None.
 *
 *****************************************************************************/


#endif