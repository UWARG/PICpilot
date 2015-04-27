/* 
 * File:   AttitudeManager.c
 * Author: Mitch
 *
 * Created on June 15, 2013, 3:40 PM
 */
 
//Include Header Files
#include "delay.h"
#include "VN100.h"
#include "InputCapture.h"
#include "OutputCompare.h"
#include "net.h"
#include "PWM.h"
#include "OrientationControl.h"
#include "AttitudeManager.h"
#include "commands.h"
#include "cameraManager.h"
#include "StartupErrorCodes.h"

#if !(PATH_MANAGER && ATTITUDE_MANAGER && COMMUNICATION_MANAGER)
#include "InterchipDMA.h"
#endif


#if !(PATH_MANAGER && ATTITUDE_MANAGER && COMMUNICATION_MANAGER)
extern PMData pmData;
extern AMData amData;
extern char newDataAvailable;
#endif


long int time = 0;
long int lastTime = 0;
long int heartbeatTimer = 0;
long int gpsTimer = 0;
char heartbeatTrigger = 0;

float* velocityComponents;

#if ATTITUDE_MANAGER

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void){
    //Timer Interrupt used for the control loops and data link
    time += 20;
    IFS0bits.T2IF = 0;
}

#endif

// Setpoints (From radio transmitter or autopilot)
int sp_PitchRate = MIDDLE_PWM;
int sp_ThrottleRate = 0;
int sp_YawRate = MIDDLE_PWM;
int sp_RollRate = MIDDLE_PWM;

int tail_OutputR;   //what the rudder used to be
int tail_OutputL;


int sp_ComputedPitchRate = 0;
//int sp_ComputedThrottleRate = 0;
int sp_ComputedRollRate = 0;
int sp_ComputedYawRate = 0;

int sp_Value = 0; //0=Roll, 1= Pitch, 2=Yaw
int sp_Type = 0; //0 = Saved Value, 1 = Edit Mode
int sp_Switch = 0;
int sp_UHFSwitch = 0;
char currentGain = 0;

int sp_PitchAngle = 0;
//float sp_YawAngle = 0;
int sp_RollAngle = 0;

//Heading Variables
int sp_Heading = 0;
int sp_HeadingRate = 0;

//Altitude Variables
int sp_Altitude = 0;
float sp_GroundSpeed = 0;

//GPS Data
int gps_Heading = 0;
float gps_GroundSpeed = 0; //NOTE: NEEDS TO BE IN METERS/SECOND. CALCULATIONS DEPEND ON THESE UNITS. GPS RETURNS KM/H.
float gps_Time = 0;
long double gps_Longitude = 0;
long double gps_Latitude = 0;
float gps_Altitude = 0;
char gps_Satellites = 0;
char gps_PositionFix = 0;
char waypointIndex = 0;
char waypointChecksum = 0;
char batteryLevel = 0;


// System outputs (get from IMU)
float imuData[3];
float imu_RollRate = 0;
float imu_PitchRate = 0;
float imu_YawRate = 0;

//IMU integration outputs
float imu_RollAngle = 0;
float imu_PitchAngle = 0;
float imu_YawAngle = 0;

int rollTrim = 0;
int pitchTrim = 0;
int yawTrim = 0;

// Control Signals (Output compare value)
int control_Roll = MIDDLE_PWM;
int control_Pitch = MIDDLE_PWM;
int control_Throttle = 0;
int control_Yaw = MIDDLE_PWM;

float scaleFactor = 1.0119; //Change this

char displayGain = 0;
int controlLevel = 0;
int lastCommandSentCode = 0;

int headingCounter = 0;
char altitudeTrigger = 0;

float refRotationMatrix[9];
float lastAltitude = 0;
long int lastAltitudeTime = 0;

char lastNumSatellites = 0;
float velocityComp_ON[3] = { 1, 0.1, 0.01}; // 1 = Scalar Mode (Velocity along the IMU X-axis, Takes into account applied rotation matrix, so if calibrated properly, no modification to frontward velocity is required), 2 = Body Reference Mode, 3 = Inertial Reference Mode
float velocityComp_OFF[3] = { 0, 0.1, 0.01};

unsigned int cameraCounter = 0;

char killingPlane = 0;

void attitudeInit() {
    //Initialize Interchip communication
    TRISFbits.TRISF3 = 0;
    LATFbits.LATF3 = 1;

    TRISDbits.TRISD14 = 0;
    LATDbits.LATD14 = 0;
    
    amData.checksum = generateAMDataChecksum();
    
    //Initialize Interchip Interrupts for Use in DMA Reset
    //Set opposite Input / Output Configuration on the PathManager
    TRISAbits.TRISA12 = 0;  //Init RA12 as Output (0), (1) is Input
    INTERCOM_1 = 0;    //Set RA12 to Output a Value of 0
    TRISAbits.TRISA13 = 0;  //Init RA13 as Output (0), (1) is Input
    INTERCOM_2 = 0;    //Set RA13 to Output a Value of 0

    TRISBbits.TRISB4 = 1;   //Init RB4 as Input (1), (0) is Output
    TRISBbits.TRISB5 = 1;   //Init RB5 as Input (1), (0) is Output
    TRISAbits.TRISA3 = 0;
    PORTAbits.RA3 = 1;

    init_SPI1();
    init_DMA0();
    init_DMA1();

    /* Initialize IMU with correct orientation matrix and filter settings */
    float filterVariance[10] = {1e-10, 1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
    VN100_initSPI();
    //IMU position matrix
    float offset[3] = {-90,90,0};
    setVNOrientationMatrix((float*)&offset);
    VN100_SPI_SetFiltMeasVar(0, (float*)&filterVariance);

    /* Initialize Input Capture and Output Compare Modules */
    if (DEBUG) {
        initPWM(0b10011111, 0b111111);
        debug("INITIALIZATION - ATTITUDE MANAGER");
    } else {
        initPWM(0b10011111, 0b111111);
    }

}

void attitudeManagerRuntime() {
    amData.checksum = generateAMDataChecksum();

    //Transfer data from PATHMANAGER CHIP
#if !PATH_MANAGER
    if (newDataAvailable){
        lastNumSatellites = gps_Satellites; //get the last number of satellites
        newDataAvailable = 0;
        char checksum = 0xAA;
        if (checksum == pmData.checksum) {
            gps_Time = pmData.time;
            gps_Heading = pmData.heading;
            gps_GroundSpeed = pmData.speed * 1000.0/3600.0; //Convert from km/h to m/s
            gps_Longitude = pmData.longitude;
            gps_Latitude = pmData.latitude;
            gps_Altitude = pmData.altitude;
            gps_Satellites = pmData.satellites;
            gps_PositionFix = pmData.positionFix;
            if (controlLevel & ALTITUDE_CONTROL_SOURCE)
                sp_Altitude = pmData.sp_Altitude;
            if (controlLevel & HEADING_CONTROL_SOURCE){
                if (gps_PositionFix){
                    sp_Heading = pmData.sp_Heading;
                }
            }
            waypointIndex = pmData.targetWaypoint;
            batteryLevel = pmData.batteryLevel;


            //turn the Velocity Compensation ON or OFF accordingly
            if (gps_Satellites >= 4 && lastNumSatellites < 4)
                VN100_SPI_WriteRegister(0, 51, 8, (unsigned long*) velocityComp_ON);
            else if (gps_Satellites < 4 && lastNumSatellites >= 4)
                VN100_SPI_WriteRegister(0, 51, 8, (unsigned long*) velocityComp_OFF);

            //newData, so feed the velocity info to the VectorNav to allow it to process and compensate accordingly
            if (gps_Satellites >= 4)
            {
                float velocity[3] = { gps_GroundSpeed, 0, 0};
                VN100_SPI_VelocityCompensationMeasurement(0, (float*)&velocity);
            }
        }
    }
//#endif

    /*****************************************************************************
     *****************************************************************************

                                INPUT CAPTURE

     *****************************************************************************
     *****************************************************************************/
        int* icTimeDiff;
        icTimeDiff = getPWMArray();

        if ((ROLL_CONTROL_SOURCE & controlLevel) == 0){
            sp_RollRate = icTimeDiff[0];
        }
        if ((PITCH_CONTROL_SOURCE & controlLevel) == 0){
            sp_PitchRate = icTimeDiff[1];
        }
        if ((THROTTLE_CONTROL_SOURCE & controlLevel) == 0)
            sp_ThrottleRate = (icTimeDiff[2]);
        

        #if(TAIL_TYPE == STANDARD_TAIL)
        {
            if ((PITCH_CONTROL_SOURCE & controlLevel) == 0)
            {
            sp_PitchRate = icTimeDiff[1];
            }
            sp_YawRate = icTimeDiff[3];
        }
        #endif
        #if(TAIL_TYPE == INV_V_TAIL)
        {
            if ((PITCH_CONTROL_SOURCE & controlLevel) == 0)
            {
                sp_PitchRate = (icTimeDiff[1] - icTimeDiff[3]) / (2 * ELEVATOR_PROPORTION);
            }
            sp_YawRate = (icTimeDiff[1] + icTimeDiff[3] ) / (2 * RUDDER_PROPORTION);
        }
        #endif

        sp_UHFSwitch = icTimeDiff[4];
//        sp_Type = icTimeDiff[5];
//        sp_Value = icTimeDiff[6];
        sp_Switch = icTimeDiff[7];

    /*****************************************************************************
     *****************************************************************************

                                IMU COMMUNICATION

     *****************************************************************************
     *****************************************************************************/



    VN100_SPI_GetRates(0, (float*) &imuData);
                           
    //Outputs in order: Roll,Pitch,Yaw
    imu_RollRate = (imuData[IMU_ROLL_RATE]); //This is a reminder for me to figure out a more elegant way to fix improper derivative control (based on configuration of the sensor), adding this negative is a temporary fix. ****MITCH REMOVED NEGATIVE
    imu_PitchRate = imuData[IMU_PITCH_RATE]; //**************MITCH HACK FIX TO CHANGE WHICH WAY IT THINKS THE PITCH ANGLE IS added negative
    imu_YawRate = imuData[IMU_YAW_RATE];
    VN100_SPI_GetYPR(0, &imuData[YAW], &imuData[PITCH], &imuData[ROLL]);
    imu_YawAngle = imuData[YAW];
    imu_PitchAngle = -imuData[PITCH]; //**************MITCH HACK FIX TO CHANGE WHICH WAY IT THINKS THE PITCH ANGLE IS added negative
    imu_RollAngle = (imuData[ROLL]); //**************MITCH HACK FIX TO CHANGE WHICH WAY IT THINKS THE PITCH ANGLE IS, added negative

    //**** Mitch put Velocity Compensation in the "newDataAvailable" section
    //Do we need this??? Might make it more accurate
//    if (gps_PositionFix == 2){
//        float velocity[3];
//        getVelocityComponents((float*)&velocity, gps_GroundSpeed, gps_Altitude, time);
//        VN100_SPI_VelocityComponentMeasurement(0, (float*)&velocity);
//
//    }

    /*****************************************************************************
     *****************************************************************************

                                 CONTROL CODE

     *****************************************************************************
     *****************************************************************************/

    if (controlLevel & ALTITUDE_CONTROL_ON){
        sp_PitchAngle = controlSignalAltitude(sp_Altitude,(int)gps_Altitude);
        if (sp_PitchAngle > MAX_PITCH_ANGLE)
            sp_PitchAngle = MAX_PITCH_ANGLE;
        if (sp_PitchAngle < -MAX_PITCH_ANGLE)
            sp_PitchAngle = -MAX_PITCH_ANGLE;
    }

    if ((THROTTLE_CONTROL_SOURCE & controlLevel) >> 4 >= 1){
        control_Throttle = sp_ThrottleRate + controlSignalThrottle(sp_Altitude, (int)gps_Altitude);
        //TODO: Fix decleration of these constants later - Maybe have a  controller.config file specific to each controller or something (make a script to generate this)
        if (control_Throttle > MAX_PWM){
            control_Throttle = MAX_PWM;
        }
        else if (control_Throttle < MIN_PWM){
            control_Throttle = MIN_PWM;
        }
    }
    else
        control_Throttle = sp_ThrottleRate;


    if (controlLevel & HEADING_CONTROL_ON){
        //Estimation of Roll angle based on heading:
        //TODO: Add if statement to use rudder for small heading changes

        while (sp_Heading > 360)
            sp_Heading -=360;
        while (sp_Heading < 0)
            sp_Heading +=360;
        // -(maxHeadingRate)/180.0,
            sp_HeadingRate = controlSignalHeading(sp_Heading, gps_Satellites>=4?gps_Heading:(int)imu_YawAngle); //changed to monitor satellites, since we know these are good values while PositionFix might be corrupt...
            //Approximating Roll angle from Heading
            sp_RollAngle = sp_HeadingRate;//(int)(atan((float)(sp_HeadingRate)) * PI/180.0);

        if (sp_RollAngle > MAX_ROLL_ANGLE)
            sp_RollAngle = MAX_ROLL_ANGLE;
        if (sp_RollAngle < -MAX_ROLL_ANGLE)
            sp_RollAngle = -MAX_ROLL_ANGLE;
    }


    // If we are getting input from the controller convert sp_xxxxRate to an sp_xxxxAngle in degrees

    if ((controlLevel & ROLL_CONTROL_SOURCE) == 0 && (controlLevel & HEADING_CONTROL_ON) == 0)
        sp_RollAngle = (int)((-sp_RollRate / ((float)SP_RANGE / MAX_ROLL_ANGLE) ));
    if ((controlLevel & PITCH_CONTROL_SOURCE) == 0 && (controlLevel & ALTITUDE_CONTROL_ON) == 0)
        sp_PitchAngle = (int)(sp_PitchRate / ((float)SP_RANGE / MAX_PITCH_ANGLE));

    //sp_YawRate = controlSignalAngles(sp_YawAngle, imu_YawAngle, kd_Accel_Yaw, -(SP_RANGE) / (maxYawAngle)) ;
    //sp_ComputedYawRate = sp_YawRate;
    // Output to servos based on requested angle and actual angle (using a gain value)
    if (controlLevel & ROLL_CONTROL_TYPE || controlLevel & HEADING_CONTROL_ON){
        sp_ComputedRollRate = controlSignalAngles(sp_RollAngle,  imu_RollAngle, ROLL, -(SP_RANGE) / (MAX_ROLL_ANGLE));
    }
    else{
        sp_ComputedRollRate = sp_RollRate;
    }

    if (controlLevel & PITCH_CONTROL_TYPE || controlLevel & ALTITUDE_CONTROL_ON){
        sp_ComputedPitchRate = controlSignalAngles(sp_PitchAngle, imu_PitchAngle, PITCH, (SP_RANGE) / (MAX_PITCH_ANGLE)); //Removed negative
    }
    else{
        sp_ComputedPitchRate = sp_PitchRate;
    }
    sp_ComputedYawRate = sp_YawRate;
    // CONTROLLER INPUT INTERPRETATION CODE
    if (sp_Switch > MIN_PWM && sp_Switch < MIN_PWM + 50) {
        unfreezeIntegral();
    } else {
        freezeIntegral();
    }




    //Feed forward Term when turning
    if (controlLevel & ALTITUDE_CONTROL_ON){
//        sp_ComputedPitchRate += abs((int)(scaleFactor * sin(deg2rad(sp_RollAngle)))) * SP_RANGE; //Sinusoidal Function
//        sp_ComputedPitchRate += abs((int)(scaleFactor * pow(sp_RollAngle,2))) * SP_RANGE; //Polynomial Function //Change this 2 to whatever
//        sp_ComputedPitchRate += abs((int)(scaleFactor * pow(sp_RollAngle,1.0/2.0))) * SP_RANGE; //Square root function
        sp_ComputedPitchRate -= abs((int)(scaleFactor * imu_RollAngle)); //Linear Function
    }

    // Control Signals (Output compare value)
    control_Roll = controlSignal((sp_ComputedRollRate / SERVO_SCALE_FACTOR), imu_RollRate, ROLL);
    control_Pitch = controlSignal((sp_ComputedPitchRate / SERVO_SCALE_FACTOR), imu_PitchRate, PITCH);
    control_Yaw = controlSignal((sp_ComputedYawRate / SERVO_SCALE_FACTOR), imu_YawRate, YAW);







    /*****************************************************************************
     *****************************************************************************

                                OUTPUT COMPARE

     *****************************************************************************
     *****************************************************************************/

    if (DEBUG) {

    }
    if (control_Roll > MAX_ROLL_PWM) {
        control_Roll = MAX_ROLL_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) > sp_RollRate - sp_ComputedRollRate) {
            setIntegralSum(ROLL, getIntegralSum(ROLL)/1.1);
        }
    }
    if (control_Roll < MIN_ROLL_PWM) {
        control_Roll = MIN_ROLL_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) < sp_RollAngle - sp_ComputedRollRate) {
            setIntegralSum(ROLL, getIntegralSum(ROLL)/1.1);
        }
    }
    if (control_Pitch > MAX_PITCH_PWM) {
        control_Pitch = MAX_PITCH_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(PITCH) * getGain(PITCH, GAIN_KI) > sp_PitchAngle - sp_ComputedPitchRate) {
            setIntegralSum(PITCH, getIntegralSum(PITCH)/1.1);
        }
    }
    if (control_Pitch < MIN_PITCH_PWM) {
        control_Pitch = MIN_PITCH_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(PITCH) * getGain(PITCH, GAIN_KI) < sp_PitchAngle - sp_ComputedPitchRate) {
            setIntegralSum(PITCH, getIntegralSum(PITCH)/1.1);
        }
    }

    if (control_Yaw > MAX_YAW_PWM)
        control_Yaw = MAX_YAW_PWM;
    if (control_Yaw < MIN_YAW_PWM)
        control_Yaw = MIN_YAW_PWM;
    
//   unsigned int cameraPWM = cameraPollingRuntime(gps_Latitude, gps_Longitude, time, &cameraCounter, imu_RollAngle, imu_PitchAngle);
     unsigned int gimbalPWM = cameraGimbalStabilization(imu_RollAngle);
     unsigned int goProGimbalPWM = goProGimbalStabilization(imu_RollAngle);
     unsigned int verticalGoProPWM = goProVerticalstabilization(imu_PitchAngle);
    // Sends the output signal to the servo motors

    //begin code for different tail configurations
    #if(TAIL_TYPE == STANDARD_TAIL)    //is a normal t-tail
    {
        tail_OutputR = control_Pitch + pitchTrim;
        tail_OutputL = control_Yaw + yawTrim;
    }
        
    #elif(TAIL_TYPE == V_TAIL)    //V-tail
    {
        //place holder
    }

    #elif(TAIL_TYPE == INV_V_TAIL)    //Inverse V-Tail
    {
        tail_OutputR =  control_Yaw * RUDDER_PROPORTION + control_Pitch * ELEVATOR_PROPORTION ;
        tail_OutputL =  control_Yaw * RUDDER_PROPORTION - control_Pitch * ELEVATOR_PROPORTION ;
    }
    #endif
    


    setPWM(1, control_Roll + rollTrim);
    setPWM(2, tail_OutputR); //Pitch
    setPWM(3, control_Throttle);
    setPWM(4, tail_OutputL); //Yaw
    //setPWM(5, cameraPWM);
    setPWM(5, goProGimbalPWM);
    setPWM(6, gimbalPWM);
//need to add outputs for goProGimbalPWM, and verticalGoProPWM

//    setPWM(7, sp_HeadingRate + MIDDLE_PWM - 20);
#endif


#if COMMUNICATION_MANAGER
    readDatalink();
    writeDatalink(DATALINK_SEND_FREQUENCY); //pwmTemp>600?0?:0xFFFFFFFF;
//    checkHeartbeat(time);
#endif
//    checkGPS(time);
}

#if COMMUNICATION_MANAGER
void readDatalink(void){
  
    struct command* cmd = popCommand();
    //TODO: Add rudimentary input validation
    if ( cmd ) {
        if (lastCommandSentCode == cmd->cmd){
            lastCommandSentCode++;
        }
        else{
            lastCommandSentCode = cmd->cmd * 100;
        }
        switch (cmd->cmd) {
            case DEBUG_TEST:             // Debugging command, writes to debug UART
                debug( (char*) cmd->data);
                break;
            case SET_PITCH_KD_GAIN:
                setGain(PITCH, GAIN_KD, *(float*)(&cmd->data));
                break;
            case SET_ROLL_KD_GAIN:
                setGain(ROLL, GAIN_KD, *(float*)(&cmd->data));
                break;
            case SET_YAW_KD_GAIN:
                setGain(YAW, GAIN_KD, *(float*)(&cmd->data));
                break;
            case SET_PITCH_KP_GAIN:
                setGain(PITCH, GAIN_KP, *(float*)(&cmd->data));
                break;
            case SET_ROLL_KP_GAIN:
                setGain(ROLL, GAIN_KP, *(float*)(&cmd->data));
                break;
            case SET_YAW_KP_GAIN:
                setGain(YAW, GAIN_KP, *(float*)(&cmd->data));
                break;
            case SET_PITCH_KI_GAIN:
                setGain(PITCH, GAIN_KI, *(float*)(&cmd->data));
                break;
            case SET_ROLL_KI_GAIN:
                setGain(ROLL, GAIN_KI, *(float*)(&cmd->data));
                break;
            case SET_YAW_KI_GAIN:
                setGain(YAW, GAIN_KI, *(float*)(&cmd->data));
                break;
            case SET_HEADING_KD_GAIN:
                setGain(HEADING, GAIN_KD, *(float*)(&cmd->data));
                break;
            case SET_HEADING_KP_GAIN:   
                setGain(HEADING, GAIN_KP, *(float*)(&cmd->data));
                break;
            case SET_HEADING_KI_GAIN:
                setGain(HEADING, GAIN_KI, *(float*)(&cmd->data));
                break;
            case SET_ALTITUDE_KD_GAIN:
                setGain(ALTITUDE, GAIN_KD, *(float*)(&cmd->data));
                break;
            case SET_ALTITUDE_KP_GAIN:
                setGain(ALTITUDE, GAIN_KP, *(float*)(&cmd->data));
                break;
            case SET_ALTITUDE_KI_GAIN:
                setGain(ALTITUDE, GAIN_KI, *(float*)(&cmd->data));
                break;
            case SET_THROTTLE_KD_GAIN:
                setGain(THROTTLE, GAIN_KD, *(float*)(&cmd->data));
                break;
            case SET_THROTTLE_KP_GAIN:
                setGain(THROTTLE, GAIN_KP, *(float*)(&cmd->data));
                break;
            case SET_THROTTLE_KI_GAIN:
                setGain(THROTTLE, GAIN_KI, *(float*)(&cmd->data));
                break;
            case SET_PATH_GAIN:
                amData.pathGain = *(float*)(&cmd->data);
                amData.command = PM_SET_PATH_GAIN;
                amData.checksum = generateAMDataChecksum();
                break;
            case SET_ORBIT_GAIN:
                amData.orbitGain = *(float*)(&cmd->data);
                amData.command = PM_SET_ORBIT_GAIN;
                amData.checksum = generateAMDataChecksum();
                break;
            case SHOW_GAIN:
                displayGain = *(char*)(&cmd->data);
                break;
            case SET_PITCH_RATE:
                sp_PitchRate = *(int*)(&cmd->data);
                break;
            case SET_ROLL_RATE:
                sp_RollRate = *(int*)(&cmd->data);
                break;
            case SET_YAW_RATE:
                sp_YawRate = *(int*)(&cmd->data);
                break;
            case SET_PITCH_ANGLE:
                sp_PitchAngle = *(int*)(&cmd->data);
                break;
            case SET_ROLL_ANGLE:
                sp_RollAngle = *(int*)(&cmd->data);
                break;
            case SET_YAW_ANGLE:
//                sp_YawAngle = *(int*)(&cmd->data);
                break;
            case SET_ALTITUDE:
                sp_Altitude = *(int*)(&cmd->data);
                break;
            case SET_HEADING:
                sp_Heading = *(int*)(&cmd->data);
                break;
            case SET_THROTTLE:
                sp_ThrottleRate = (*(int*)(&cmd->data) * MAX_PWM / 100);
                break;
            case SET_AUTONOMOUS_LEVEL:
                controlLevel = *(int*)(&cmd->data);
                break;
            case SET_ANGULAR_WALK_VARIANCE:
                setAngularWalkVariance(*(float*)(&cmd->data));
                break;
            case SET_GYRO_VARIANCE:
                setGyroVariance(*(float*)(&cmd->data));
                break;
            case SET_MAGNETIC_VARIANCE:
                setMagneticVariance(*(float*)(&cmd->data));
                break;
            case SET_ACCEL_VARIANCE:
                setAccelVariance(*(float*)(&cmd->data));
                break;
            case SET_SCALE_FACTOR:
                scaleFactor = *(float*)(&cmd->data);
                break;
            case CALIBRATE_ALTIMETER:
                amData.calibrationHeight = *(float*)(&cmd->data);
                amData.command = PM_CALIBRATE_ALTIMETER;
                amData.checksum = generateAMDataChecksum();
                break;
            case CLEAR_WAYPOINTS:
                amData.waypoint.id = (*(char *)(&cmd->data)); //Dummy Data
                amData.command = PM_CLEAR_WAYPOINTS;
                amData.checksum = generateAMDataChecksum();
                break;
            case REMOVE_WAYPOINT:
                amData.waypoint.id = (*(char *)(&cmd->data));
                amData.command = PM_REMOVE_WAYPOINT;
                amData.checksum = generateAMDataChecksum();
                break;
            case SET_TARGET_WAYPOINT:
                amData.waypoint.id = *(char *)(&cmd->data);
                amData.command = PM_SET_TARGET_WAYPOINT;
                amData.checksum = generateAMDataChecksum();
                break;
            case RETURN_HOME:
                amData.command = PM_RETURN_HOME;
                amData.checksum = generateAMDataChecksum();
                break;
            case CANCEL_RETURN_HOME:
                amData.command = PM_CANCEL_RETURN_HOME;
                amData.checksum = generateAMDataChecksum();
                break;
            case SEND_HEARTBEAT:
                heartbeatTimer = time;
                break;
            case TRIGGER_CAMERA:
                triggerCamera(*(unsigned int*)(&cmd->data));
                break;
            case SET_TRIGGER_DISTANCE:
                setTriggerDistance(*(float*)(&cmd->data));
                break;
            case SET_GIMBLE_OFFSET:
                setGimbalOffset(*(unsigned int*)(&cmd->data));
                break;
            case KILL_PLANE:
                if (*(int*)(&cmd->data) == 1234)
                    killingPlane = 1;
                break;
            case UNKILL_PLANE:
                if (*(int*)(&cmd->data) == 1234)
                    killingPlane = 0;
                break;

            case NEW_WAYPOINT:
                amData.waypoint.altitude = (*(WaypointWrapper*)(&cmd->data)).altitude;
                amData.waypoint.id = (*(WaypointWrapper*)(&cmd->data)).id;
                amData.waypoint.latitude = (*(WaypointWrapper*)(&cmd->data)).latitude;
                amData.waypoint.longitude = (*(WaypointWrapper*)(&cmd->data)).longitude;
                amData.waypoint.radius = (*(WaypointWrapper*)(&cmd->data)).radius;
                amData.command = PM_NEW_WAYPOINT;
                amData.checksum = generateAMDataChecksum();
                break;
            case INSERT_WAYPOINT:
                amData.waypoint.altitude = (*(WaypointWrapper*)(&cmd->data)).altitude;
                amData.waypoint.latitude = (*(WaypointWrapper*)(&cmd->data)).latitude;
                amData.waypoint.longitude = (*(WaypointWrapper*)(&cmd->data)).longitude;
                amData.waypoint.radius = (*(WaypointWrapper*)(&cmd->data)).radius;
                amData.waypoint.nextId = (*(WaypointWrapper*)(&cmd->data)).nextId;
                amData.waypoint.previousId = (*(WaypointWrapper*)(&cmd->data)).previousId;
                amData.command = PM_INSERT_WAYPOINT;
                amData.checksum = generateAMDataChecksum();
                break;
            case SET_RETURN_HOME_COORDINATES:
                amData.waypoint.altitude = (*(WaypointWrapper*)(&cmd->data)).altitude;
                amData.waypoint.latitude = (*(WaypointWrapper*)(&cmd->data)).latitude;
                amData.waypoint.longitude = (*(WaypointWrapper*)(&cmd->data)).longitude;
                amData.command = PM_SET_RETURN_HOME_COORDINATES;
                amData.checksum = generateAMDataChecksum();
                break;
            case TARE_IMU:
                adjustVNOrientationMatrix((float*)(&cmd->data));
                break;
            case SET_IMU:
                setVNOrientationMatrix((float*)(&cmd->data));
                break;
            default:
                break;
        }
        destroyCommand( cmd );
    }
 
}
int writeDatalink(long frequency){
 
    if (time - lastTime > frequency) {
        lastTime = time;
    
        struct telem_block* statusData = createTelemetryBlock();//getDebugTelemetryBlock();
  
        statusData->lat = gps_Latitude;
        statusData->lon = gps_Longitude;
        statusData->millis = gps_Time;
        statusData->pitch = imu_PitchAngle;
        statusData->roll = imu_RollAngle;
        statusData->yaw = imu_YawAngle;
        statusData->pitchRate = imu_PitchRate;
        statusData->rollRate = imu_RollRate;
        statusData->yawRate = imu_YawRate;
        statusData->pitch_gain = getGain(displayGain, GAIN_KD);
        statusData->roll_gain = getGain(displayGain, GAIN_KP);
        statusData->yaw_gain = getGain(displayGain, GAIN_KI);
        statusData->heading = gps_Heading;
        statusData->groundSpeed = gps_GroundSpeed;
        statusData->pitchSetpoint = sp_PitchAngle;
        statusData->rollSetpoint = sp_RollAngle;
        statusData->headingSetpoint = sp_Heading;
        statusData->throttleSetpoint = (sp_ThrottleRate * 100) / MAX_PWM;
        statusData->altitudeSetpoint = sp_Altitude;
        statusData->altitude = gps_Altitude;
        statusData->cPitchSetpoint = sp_PitchRate;
        statusData->cRollSetpoint = sp_RollRate;
        statusData->cYawSetpoint = sp_YawRate;
        statusData->lastCommandSent = lastCommandSentCode;
        statusData->errorCodes = getErrorCodes() + ((sp_UHFSwitch < -429)<<11);
        statusData->cameraStatus = cameraCounter;
        statusData->waypointIndex = waypointIndex;
        statusData->editing_gain = displayGain + ((sp_Switch > 380) << 4);
        statusData->gpsStatus = gps_Satellites + (gps_PositionFix << 4);
        statusData->batteryLevel = batteryLevel;
        

        if (BLOCKING_MODE) {
            sendTelemetryBlock(statusData);
            destroyTelemetryBlock(statusData);
        } else {
            return pushOutboundTelemetryQueue(statusData);
        }
    }
         
    return 0;

}
#endif

void adjustVNOrientationMatrix(float* adjustment){

    adjustment[0] = deg2rad(adjustment[0]);
    adjustment[1] = deg2rad(adjustment[1]);
    adjustment[2] = deg2rad(adjustment[2]);

    float matrix[9];
    VN100_SPI_GetRefFrameRot(0, (float*)&matrix);
    
    float refRotationMatrix[9] = {cos(adjustment[1]) * cos(adjustment[2]), -cos(adjustment[1]) * sin(adjustment[2]), sin(adjustment[1]),
        sin(deg2rad(adjustment[0])) * sin(adjustment[1]) * cos(adjustment[2]) + sin(adjustment[2]) * cos(adjustment[0]), -sin(adjustment[0]) * sin(adjustment[1]) * sin(adjustment[2]) + cos(adjustment[2]) * cos(adjustment[0]), -sin(adjustment[0]) * cos(adjustment[1]),
        -cos(deg2rad(adjustment[0])) * sin(adjustment[1]) * cos(adjustment[2]) + sin(adjustment[2]) * sin(adjustment[0]), cos(adjustment[0]) * sin(adjustment[1]) * sin(adjustment[2]) + cos(adjustment[2]) * sin(adjustment[0]), cos(adjustment[0]) * cos(adjustment[1])};

    int i = 0;
    for (i = 0; i < 9; i++){
        refRotationMatrix[i] += matrix[i];
    }

    VN100_SPI_SetRefFrameRot(0, (float*)&refRotationMatrix);
    VN100_SPI_WriteSettings(0);
    VN100_SPI_Reset(0);

}

void setVNOrientationMatrix(float* angleOffset){
    //angleOffset[0] = x, angleOffset[1] = y, angleOffset[2] = z
    angleOffset[0] = deg2rad(angleOffset[0]);
    angleOffset[1] = deg2rad(angleOffset[1]);
    angleOffset[2] = deg2rad(angleOffset[2]);
    
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

// TODO: make me a real checksum!
char generateAMDataChecksum(void){
    return 0xAB;
}

void checkHeartbeat(long int cTime){
    if (cTime - heartbeatTimer > HEARTBEAT_TIMEOUT){
//        amData.command = PM_RETURN_HOME;
//        amData.checksum = generateAMDataChecksum();
    }
    else if (cTime - heartbeatTimer > HEARTBEAT_KILL_TIMEOUT){
//        killingPlane = 1;
    }
}

//void checkGPS(long int cTime){
//    if (cTime - gpsTimer > GPS_TIMEOUT){
//        gpsTimer = cTime;
//        if (gps_PositionFix == 0)
//            killingPlane = 1;
//    }
//}

void getVelocityComponents(float* velocityComponents, float groundSpeed, float altitude, long int time){
    long int timeChange = time - lastAltitudeTime;
    lastAltitudeTime = time;

    float altitudeChange = (altitude - lastAltitude)/ timeChange;
    lastAltitude = altitude;

    //Matrix Multiplication...the VN100 doesn't do this, so we must do it instead.
    //The velocity vector is:
    //x:groundSpeed * cos(gps_Heading - imu_YawAngle);
    //y:groundSpeed * sin(gps_Heading - imu_YawAngle);
    //z:altitudeChange;
    velocityComponents[0] = (groundSpeed * cos(deg2rad(gps_Heading - imu_YawAngle))) * refRotationMatrix[0] + (groundSpeed * sin(gps_Heading - imu_YawAngle)) * refRotationMatrix[1] +  altitudeChange * refRotationMatrix[2];
    velocityComponents[1] = (groundSpeed * cos(deg2rad(gps_Heading - imu_YawAngle))) * refRotationMatrix[3] + (groundSpeed * sin(gps_Heading - imu_YawAngle)) * refRotationMatrix[4] +  altitudeChange * refRotationMatrix[5];
    velocityComponents[2] = (groundSpeed * cos(deg2rad(gps_Heading - imu_YawAngle))) * refRotationMatrix[6] + (groundSpeed * sin(gps_Heading - imu_YawAngle)) * refRotationMatrix[7] +  altitudeChange * refRotationMatrix[8];
}
