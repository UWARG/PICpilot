/* 
 * File:   AttitudeManager.h
 * Author: Chris Hajduk
 *
 * Created on January 26, 2014, 5:43 PM
 */

#ifndef ATTITUDEMANAGER_H
#define	ATTITUDEMANAGER_H

//Header Files
#include "main.h"

//Constants involving attitude management
#define STABILIZATION_CONTROL 1
#define ORIENTATION_CONTROL 1
#define HEADING_CONTROL 1
#define ALTITUDE_CONTROL 1
#define THROTTLE_CONTROL 1

//Levels of control
#define CONTROLLER_RATE_CONTROL 0 //Level 0 is full manual control
#define CONTROLLER_ANGLE_CONTROL 1
#define CONTROLLER_ALTITUDE_CONTROL 2
#define CONTROLLER_HEADING_CONTROL 3
#define GROUND_STATION_RATE_CONTROL 4
#define GROUND_STATION_ANGLE_CONTROL 5
#define GROUND_STATION_ALTITUDE_CONTROL 6
#define GROUND_STATION_HEADING_CONTROL 7
#define GROUND_STATION_THROTTLE_CONTROL 8
#define FULL_AUTONOMY 9 //Level 9 is the highest level of autonomy

//Defined Orientation Parameter constants
#define YAW     0
#define PITCH   1
#define ROLL    2
#define HEADING 3
#define ALTITUDE 4
#define THROTTLE 5

//Defined in the opposite order for rates of rotation
//Be careful not to confuse the constants
#define YAW_RATE     2
#define PITCH_RATE   1
#define ROLL_RATE    0



/* FUNCTION PROTOTYPES */
/*****************************************************************************
 * Function: void attitudeInit(void)
 *
 * Preconditions: None.
 *
 * Overview: This function initializes the IMU, SPI, Input Capture, and Output
 * Compare modules or devices.
 *
 * Input:   None.
 *
 * Output:  None.
 *
 *****************************************************************************/
void attitudeInit(void);

/*****************************************************************************
 * Function: void attitudeManagerRuntime(void)
 *
 * Preconditions: attitudeInit() must have been called before hand.
 *
 * Overview: This function is responsible for the continuous monitoring of the
 * orientation of the plane. It contains code that retrieves information from
 * the remote control, or the path manager. It also retrieves information from
 * the IMU. It then compares the data to provide an optimal output the the
 * aircrafts control surfaces. It also sends data to the data link manager at
 * the end of the cycle.
 *
 * Input:   None.
 *
 * Output:  None.
 *
 *****************************************************************************/
void attitudeManagerRuntime(void);

#if COMMUNICATION_MANAGER

/*****************************************************************************
 * Function: void readDatalink(void);
 *
 * Preconditions: The datalink must have been initialized to use this properly.
 *
 * Overview: This function is responsible for reading commands from the datalink.
 * Each command has an associated function identified by the "cmd" parameter of the
 * data struct. Additional functions may be added to the switch statement (up to
 * 256 possible commands).
 *
 * Input:   None.
 *
 * Output:  None.
 *
 *****************************************************************************/
void readDatalink(void);

/*****************************************************************************
 * Function: int writeDatalink(long frequency);
 *
 * Preconditions: The datalink must have been initialized to use this properly.
 *
 * Overview: This function is responsible for writing to the datalink at a
 * certain frequency (in milliseconds). The time value is dependent on the timer interrupt.
 *
 * Input:   The rate at which data is added to the datalink output queue.
 *
 * Output:  An error code indicating if the data was added to the queue successfully.
 *
 *****************************************************************************/
int writeDatalink(long frequency);
#endif

/*****************************************************************************
 * Function: void tareVN100(void);
 *
 * Preconditions: The VN100 module, and the SPI2 interface must have already been initialized.
 *
 * Overview: This function takes the current position of the VN100 model, and applies
 * it into an orientation matrix which removes any bias.
 *
 * Input:   None.
 *
 * Output:  None.
 *
 *****************************************************************************/
void tareVN100(void);

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
void setVNOrientationMatrix(float* angleOffset);
#endif	/* ATTITUDEMANAGER_H */

