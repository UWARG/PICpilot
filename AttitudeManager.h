/* 
 * File:   AttitudeManager.h
 * Author: Chris Hajduk
 *
 * Created on January 26, 2014, 5:43 PM
 */

#ifndef ATTITUDEMANAGER_H
#define	ATTITUDEMANAGER_H

//Constants involving attitude management
#define STABILIZATION_CONTROL 1
#define ORIENTATION_CONTROL 1
#define HEADING_CONTROL 1

//Defined Orientation Angle constants
#define YAW     0
#define PITCH   1
#define ROLL    2
#define HEADING 3

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



#endif	/* ATTITUDEMANAGER_H */

