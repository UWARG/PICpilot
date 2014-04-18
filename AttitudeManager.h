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

//Bit masks for levels of control
#define PITCH_CONTROL_TYPE 0b00000001 //Pitch Rate(0) or Pitch Angles(1)
#define PITCH_CONTROL_SOURCE 0b00000010 //Controller(0) or Ground Station(1)
#define ROLL_CONTROL_TYPE 0b00000100 //Roll Rates(0) or Roll Angles(1)
#define ROLL_CONTROL_SOURCE 0b00001000 //Controller(0) or Ground Station(1)
#define THROTTLE_CONTROL_SOURCE 0b00110000 //Controller(0) or Ground Station(1) or Autopilot(2)(Controlled by the GroundSpeed).
#define ALTITUDE_CONTROL_SOURCE 0b01000000 //Ground Station(0) or Autopilot(1)
#define ALTITUDE_CONTROL_ON 0b10000000 //Off(0) or On(1)
#define HEADING_CONTROL_SOURCE 0b0000000100000000 // Ground Station(0) or Autopilot(1)
#define HEADING_CONTROL_ON 0b0000001000000000 //Off(0) or On(1)

//Defined Orientation Parameter constants
#define YAW     0
#define PITCH   1
#define ROLL    2
#define HEADING 3
#define ALTITUDE 4
#define THROTTLE 5

//Defined in the opposite order for rates of rotation
//Be careful not to confuse the constants
#define IMU_YAW_RATE     2
#define IMU_PITCH_RATE   1
#define IMU_ROLL_RATE    0

#define MAX_ROLL_PWM UPPER_PWM - 20
#define MIN_ROLL_PWM LOWER_PWM + 40
#define MAX_PITCH_PWM UPPER_PWM - 25
#define MIN_PITCH_PWM LOWER_PWM + 25
#define MAX_YAW_PWM UPPER_PWM - 25
#define MIN_YAW_PWM LOWER_PWM + 25

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
void adjustVNOrientationMatrix(float* adjustment);

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

//TODO: Add descriptions for these

void setAngularWalkVariance(float variance);
void setGyroVariance(float variance);
void setMagneticVariance(float variance);
void setAccelVariance(float variance);
char generateAMDataChecksum(void);
#endif	/* ATTITUDEMANAGER_H */

