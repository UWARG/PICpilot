/* 
 * File:   OrientationControl.h
 * Author: Chris Hajduk
 *
 * Created on October 29, 2013, 10:46 PM
 */

//Global Variables

#define GAIN_KD 0
#define GAIN_KP 1
#define GAIN_KI 2

//Maximum rates for PID loop outputs
#define MAX_HEADING_RATE 10.0 //Max heading change of 10 degrees per second
#define MAX_ROLL_ANGLE 40.0 // max allowed roll angle in degrees
#define MAX_PITCH_ANGLE 15.0
#define MAX_SPEED 17.0 // ???m/s

// A scaling factor used in the PID control loops
#define SERVO_SCALE_FACTOR (-(UPPER_PWM - MIDDLE_PWM) / 45)
#define ALTITUDE_PITCH_SCALE_FACTOR 1 //0.1 degrees per meter in altitude change
#define HEADING_ROLL_SCALE_FACTOR MAX_ROLL_ANGLE/360.0
#define THROTTLE_SCALE_FACTOR SP_RANGE/MAX_PITCH_ANGLE



//Function Prototypes
//TODO: Add function comments here
float controlSignalThrottle(float setpoint, float output, float time);
float controlSignalAltitude(float sp_Altitude,float gps_Altitude, float time);
/*****************************************************************************
 * Function: float controlSignalHeading(float setpoint, float output, float time)
 *
 * Preconditions: None.
 *
 * Overview: This function is responsible for the heading of the plane. It
 * contains the equations that model a PID control system. It calculates the
 * proportional, integral, and derivative terms of the control signal. The setpoint (target value
 * required by the system) and the output (current state of the system) are both
 * inputs to this function in terms of angles (deg) ranging from 0 to 359 degrees (as per compass bearings).
 *
 * Input:   float setpoint -> The target value that is required by the system.
 *              (Input from the controller or path manager)
 *          float output -> The acutal value of the system (Data from the GPS)
 *          float time -> The current time as of the last system cycle.
 *
 * Output:  float -> An integer value representing the output of the system
 *              (heading rate). This value is in degrees per second.
 *
 *****************************************************************************/
float controlSignalHeading(float setpoint, float output, float time);

/*****************************************************************************
 * Function: int controlSignalAngles(float setpoint, float output, unsigned char type, float SERVO_SCALE_FACTOR_ANGLES, float time)
 *
 * Preconditions: None.
 *
 * Overview: This function is responsible for the orientation of the plane. It
 * contains the equations that model a PID control system. It calculates the
 * proportional and integral term of the control signal. The setpoint (target value
 * required by the system) and the output (current state of the system) are both
 * inputs to this function in terms of angles (deg or rad). The units depend
 * on the value of SERVO_SCALE_FACTOR_ANGLES which can be changed for various units.
 *
 * Input:   float setpoint -> The target value that is required by the system.
 *              (Input from the controller)
 *          float output -> The acutal value of the system (Data from the IMU)
 *          unsigned char type -> The value indicating the pitch, roll, or yaw
 *              components to be calculated. See main.h for their definitions.
 *          float SERVO_SCALE_FACTOR_ANGLES -> The value that scales the input
 *              values to the return value. See above for the definition.
 *          float time -> The current time as of the last system cycle.
 *
 * Output:  int -> An integer value representing the output of the system
 *              (angular rate). This value is in arbitrary timer ticks that vary
 *              based on the microcontroller (oscillator) being used.
 *
 *****************************************************************************/

int controlSignalAngles(float setpoint, float output, unsigned char type, float SERVO_SCALE_FACTOR_ANGLES, float time);

/*****************************************************************************
 * Function: int controlSignal(float setpoint, float output, unsigned char type)
 *
 * Preconditions: None.
 *
 * Overview: This function is responsible for the angular rates of the plane. It
 * contains the differential equations that are part of the PID control system.
 * It calculates the derivative term of the control signal. The setpoint (target value
 * required by the system) and the output (current state of the system) are both
 * inputs to this function in terms of angles (deg/s or rad/s). The units depend
 * on the value of SERVO_SCALE_FACTOR (see above) which can be changed for various units.
 *
 * Input:   float setpoint -> The target value that is required by the system.
 *              (Input from the controller)
 *          float output -> The acutal value of the system (Data from the IMU)
 *          unsigned char type -> The value indicating the pitch, roll, or yaw
 *              components to be calculated. See main.h for their definitions.
 *
 * Output:  int -> An integer value representing the output of the system
 *              (timer tick values). The timer tick values represent the width of
 *              a square wave. It can vary based on the microcontroller
 *              (oscillator) being used.
 *
 *****************************************************************************/

int controlSignal(float setpoint, float output, unsigned char type);

/*****************************************************************************
 * Function: void freezeIntegral()
 *
 * Preconditions: None.
 *
 * Overview: This function is used to prevent the integrator from building up in
 * the PID loop. If the integrator is left running, a tragic control overshoot
 * could result. This is often used if the calculations are still being processed,
 * when the plane is in manual mode (not conforming to the control signals).
 *
 * Input:   None.
 *
 * Output:  None.
 *
 *****************************************************************************/

void freezeIntegral();

/*****************************************************************************
 * Function: void unfreezeIntegral()
 *
 * Preconditions: None.
 *
 * Overview: This function is used to renable the integrator after it was turned
 * off by the function above. If the integrator is left left frozen, the integral
 * control signal code will have no effect. This is often used if control is
 * switched back to the autopilot from manual control.
 *
 * Input:   None.
 *
 * Output:  None.
 *
 *****************************************************************************/

void unfreezeIntegral();

/*****************************************************************************
 * Function: void setIntegralSum(unsigned char YPR, float value)
 *
 * Preconditions: None.
 *
 * Overview: This function is used set the value of the integrator (from the 
 * PID loops). This is often used if the integrator builds up past a limit and
 * needs to be reduced.
 *
 * Input:   unsigned char YPR -> The value indicating the pitch, roll, or yaw
 *              components. See main.h for their definitions.
 *          float value -> The value to set the integral sum to.
 *
 * Output:  None.
 *
 *****************************************************************************/

void setIntegralSum(unsigned char YPR, float value);

/*****************************************************************************
 * Function: float getIntegralSum(unsigned char YPR)
 *
 * Preconditions: None.
 *
 * Overview: This function is used get the value of the integrator (from the
 * PID loops). This is often used to check the integrator for testing and
 * automatic checking/limiting.
 *
 * Input:   unsigned char YPR -> The value indicating the pitch, roll, or yaw
 *              components. See main.h for their definitions.
 *
 * Output:  float -> The value of the requested integral sum.
 *
 *****************************************************************************/

float getIntegralSum(unsigned char YPR);

/*****************************************************************************
 * Function: float getGain(unsigned char YPR, unsigned char type)
 *
 * Preconditions: None.
 *
 * Overview: This function is used get the value of the gains (from the
 * PID loops). This is often used to check the gains for testing and
 * automatic checking/limiting.
 *
 * Input:   unsigned char YPR -> The value indicating the pitch, roll, or yaw
 *              components. See main.h for their definitions.
 *          unsigned char type -> The value indicating the type of gain (KP,KI,KD)
 *              to return. See the definitions at the top of this header file.
 *
 * Output:  float -> The value of the requested gain.
 *
 *****************************************************************************/

float getGain(unsigned char YPR, unsigned char type);

/*****************************************************************************
 * Function: void setGain(unsigned char YPR, unsigned char type, float value)
 *
 * Preconditions: None.
 *
 * Overview: This function is used set the value of the gains (for the
 * PID loops). This is often used to alter the gains for testing and
 * automatic checking/limiting.
 *
 * Input:   unsigned char YPR -> The value indicating the pitch, roll, or yaw
 *              components. See main.h for their definitions.
 *          unsigned char type -> The value indicating the type of gain (KP,KI,KD)
 *              to return. See the definitions at the top of this header file.
 *          float -> The value of the selected gain.
 *
 * Output:  None.
 *
 *****************************************************************************/

void setGain(unsigned char YPR, unsigned char type, float value);



