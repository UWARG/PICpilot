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
#include "PWM.h"
#include "AttitudeManager.h"
#include "commands.h"
#include "cameraManager.h"
#include "Probe_Drop.h"
#include "StartupErrorCodes.h"
#include "main.h"
#include "InterchipDMA.h"
#include "ProgramStatus.h"
#include <string.h>

extern PMData pmData;
extern AMData amData;
extern char DMADataAvailable;

long int lastTime = 0;
long int heartbeatTimer = 0;
long int UHFTimer = 0;
long int gpsTimer = 0;

float* velocityComponents;

// Setpoints (From radio transmitter or autopilot)
int sp_PitchRate = 0;
int sp_ThrottleRate = MIN_PWM;
int sp_FlapRate = MIN_PWM;
int sp_YawRate = 0;
int sp_RollRate = 0;

int tail_OutputR;   //what the rudder used to be
int tail_OutputL;


int sp_ComputedPitchRate = 0;
//int sp_ComputedThrottleRate = 0;
int sp_ComputedRollRate = 0;
int sp_ComputedYawRate = 0;

char currentGain = 0;

int sp_PitchAngle = 0;
int ctrl_PitchAngle = 0;
//float sp_YawAngle = 0;
int sp_RollAngle = 0;
int ctrl_RollAngle = 0;

//Heading Variables
int sp_Heading = 0;
int ctrl_Heading = 0;

//Altitude Variables
int sp_Altitude = 0;
int ctrl_Altitude = 0;
float sp_GroundSpeed = 0;

//GPS Data
int gps_Heading = 0;
float gps_GroundSpeed = 0; //NOTE: NEEDS TO BE IN METERS/SECOND. CALCULATIONS DEPEND ON THESE UNITS. GPS RETURNS KM/H.
float gps_Time = 0;
long double gps_Longitude = 0;
long double gps_Latitude = 0;
float gps_Altitude = 0;
float airspeed = 0;
char gps_Satellites = 0;
char gps_PositionFix = 0;
float pmOrbitGain = 0;
float pmPathGain = 0;
char waypointIndex = 0;
float waypointChecksum = 0;
char pathFollowing = 0;
char waypointCount = 0;
int batteryLevel1 = 0;
int batteryLevel2 = 0;
char lastProbeDrop = 0;


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

//RC Input Signals (Input Capture Values)
int input_RC_RollRate = 0;
int input_RC_PitchRate = 0;
int input_RC_Throttle = 0;
int input_RC_Flap = 0;
int input_RC_YawRate = 0;
int input_RC_Aux1 = 0; //0=Roll, 1= Pitch, 2=Yaw
int input_RC_Aux2 = 0; //0 = Saved Value, 1 = Edit Mode
int input_RC_Switch1 = 0;

//Ground Station Input Signals
int input_GS_Roll = 0;
int input_GS_Pitch = 0;
int input_GS_Throttle = 0;
int input_GS_Flap = 0;
int input_GS_Yaw = 0;
int input_GS_RollRate = 0;
int input_GS_PitchRate = 0;
int input_GS_YawRate = 0;
int input_GS_Altitude = 0;
int input_GS_Heading = 0;

int input_AP_Altitude = 0;
int input_AP_Heading = 0;

//PID Global Variable Storage Values
int rollPID, pitchPID, throttlePID, yawPID, flapPID;

float scaleFactor = 20; //Change this

char displayGain = 0;
int controlLevel = 0;
int lastCommandSentCode[COMMAND_HISTORY_SIZE];
int lastCommandCounter = 0;

int headingCounter = 0;
char altitudeTrigger = 0;

float refRotationMatrix[9];
float lastAltitude = 0;
long int lastAltitudeTime = 0;

char lastNumSatellites = 0;

unsigned int cameraCounter = 0;

char show_scaled_pwm = 1;

void attitudeInit() {
    setProgramStatus(INITIALIZATION);
    //Initialize Timer
    initTimer4();

    //Initialize Interchip communication
    TRISFbits.TRISF3 = 0;
    LATFbits.LATF3 = 1;

    //the line below sets channel 7 IC to be an output. Used to be for DMA. Should
    //investigate DMA code to see why this was used in the first place, and if it'll have
    //any negative consequences. Commenting out for now. Serge Feb 7 2017
    //TRISDbits.TRISD14 = 0;
    LATDbits.LATD14 = 0;

    amData.checkbyteDMA = generateAMDataDMACheckbyte();

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


    /* Initialize Input Capture and Output Compare Modules */
#if DEBUG
    initPWM(0b11111111, 0b11111111);
#else
    initPWM(0b11111111, 0b11111111);
#endif
    /*  *****************************************************************
     *  IMU
     *  Initialize IMU with correct orientation matrix and filter settings
     *  *****************************************************************
     */
    //In order: Angular Walk, Angular Rate x 3, Magnetometer x 3, Acceleration x 3
    float filterVariance[10] = {1e-9, 1e-9, 1e-9, 1e-9, 1, 1, 1, 1e-4, 1e-4, 1e-4}; //  float filterVariance[10] = {1e-10, 1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
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

        setVNOrientationMatrix((float*)&offset);
        VN100_SPI_SetFiltMeasVar(0, (float*)&filterVariance);
        setSensorStatus(VECTORNAV, SENSOR_INITIALIZED & TRUE);
    }
    else{
        setSensorStatus(VECTORNAV, SENSOR_CONNECTED & FALSE);
    }
#if DEBUG
        debug("Datalink Initialized");
#endif

    initDataLink();
    if (checkDataLinkConnection()){
        setSensorStatus(XBEE, SENSOR_INITIALIZED & TRUE);
    }
    initialization();
    setProgramStatus(MAIN_EXECUTION);
}


char checkDMA(){
    //Transfer data from PATHMANAGER CHIP
    lastNumSatellites = gps_Satellites; //get the last number of satellites
    DMADataAvailable = 0;
    if (generatePMDataDMAChecksum1() == pmData.checkbyteDMA1 && generatePMDataDMAChecksum2() == pmData.checkbyteDMA2) {
        gps_Time = pmData.time;
        input_AP_Altitude = pmData.sp_Altitude;
        gps_Satellites = pmData.satellites;
        gps_PositionFix = pmData.positionFix;
        waypointIndex = pmData.targetWaypoint;
        batteryLevel1 = pmData.batteryLevel1;
        batteryLevel2 = pmData.batteryLevel2;
        waypointCount = pmData.waypointCount;
        waypointChecksum = pmData.waypointChecksum;
        pathFollowing = pmData.pathFollowing;
        airspeed = pmData.airspeed;
        pmOrbitGain = pmData.pmOrbitGain;
        pmPathGain = pmData.pmPathGain;

        //Check if this data is new and requires action or if it is old and redundant
        if (gps_Altitude == pmData.altitude && gps_Heading == pmData.heading && gps_GroundSpeed == pmData.speed && gps_Latitude == pmData.latitude && gps_Longitude == pmData.longitude){
            return FALSE;
        }

        gps_Heading = pmData.heading;
        gps_GroundSpeed = pmData.speed * 1000.0/3600.0; //Convert from km/h to m/s
        gps_Longitude = pmData.longitude;
        gps_Latitude = pmData.latitude;
        gps_Altitude = pmData.altitude;


//        if (pmData.dropProbe > lastProbeDrop){
//            lastProbeDrop = pmData.dropProbe;
//            char availableProbes = getProbeStatus();
//            int i = 0;
//            for (i = 0; i < MAX_PROBE; i++){
//                if (availableProbes & (1 << i)){
//                    dropProbe(pmData.dropProbe);
//                }
//            }
//
//        }
        if (gps_PositionFix){
            input_AP_Heading = pmData.sp_Heading;
        }
        return TRUE;
    }
    else{
//        INTERCOM_2 = 1;
        SPI1STATbits.SPIEN = 0; //Disable SPI1
        DMA0CONbits.CHEN = 0; //Disable DMA0 channel
        DMA1CONbits.CHEN = 0; //Disable DMA1 channel
        while(SPI1STATbits.SPIRBF) { //Clear SPI1
            SPI1BUF;
        }
//        INTERCOM_2 = 0;
//        while(INTERCOM_4);
        init_SPI1(); // Restart SPI
        init_DMA0(); // Restart DMA0
        init_DMA1(); // Restart DMA1
        DMA1REQbits.FORCE = 1;
        while (DMA1REQbits.FORCE == 1);
        return FALSE;
    }

}

float getAltitude(){
    return gps_Altitude;
}
int getHeading(){
    return gps_Heading;
}
long double getLongitude(){
    return gps_Longitude;
}
long double getLatitude(){
    return gps_Latitude;
}
float getRoll(){
    return imu_RollAngle;
}
float getPitch(){
    return imu_PitchAngle;
}
float getYaw(){
    return imu_YawAngle;
}
float getRollRate(){
    return imu_RollRate;
}
float getPitchRate(){
    return imu_PitchRate;
}
float getYawRate(){
    return imu_YawRate;
}
int getRollAngleSetpoint(){
    return sp_RollAngle;
}
int getPitchAngleSetpoint(){
    return sp_PitchAngle;
}
int getPitchRateSetpoint(){
    return sp_PitchRate;
}
int getRollRateSetpoint(){
    return sp_RollRate;
}
int getYawRateSetpoint(){
    return sp_YawRate;
}
int getThrottleSetpoint(){
    return sp_ThrottleRate;
}
int getFlapSetpoint(){
    return sp_FlapRate;
}
int getAltitudeSetpoint(){
    return sp_Altitude;
}
int getHeadingSetpoint(){
    return sp_Heading;
}

void setPitchAngleSetpoint(int setpoint){
    sp_PitchAngle = setpoint;
}
void setRollAngleSetpoint(int setpoint){
    sp_RollAngle = setpoint;
}
void setPitchRateSetpoint(int setpoint){
    sp_PitchRate = setpoint;
}
void setRollRateSetpoint(int setpoint){
    sp_RollRate = setpoint;
}
void setYawRateSetpoint(int setpoint){
    sp_YawRate = setpoint;
}
void setThrottleSetpoint(int setpoint){
    sp_ThrottleRate = setpoint;
}
void setFlapSetpoint(int setpoint){
    sp_FlapRate = setpoint;
}
void setAltitudeSetpoint(int setpoint){
    sp_Altitude = setpoint;
}
void setHeadingSetpoint(int setpoint){
    sp_Heading = setpoint;
}

void inputCapture(){
    int* channelIn;
    channelIn = getPWMArray(getTime());
    
    inputMixing(channelIn, &input_RC_RollRate, &input_RC_PitchRate, &input_RC_Throttle, &input_RC_YawRate);
    
#if VEHICLE_TYPE == FIXED_WING
    if (getControlPermission(FLAP_CONTROL_SOURCE, FLAP_RC_SOURCE,FLAP_CONTROL_SOURCE_SHIFT)) {
        input_RC_Flap = channelIn[FLAP_IN_CHANNEL - 1];
    }
#endif
    // Switches and Knobs
//        sp_Type = channelIn[5];
//        sp_Value = channelIn[6];
    input_RC_Switch1 = channelIn[AUTOPILOT_ACTIVE_IN_CHANNEL - 1];

    //Controller Input Interpretation Code
    if (input_RC_Switch1 > MIN_PWM && input_RC_Switch1 < MIN_PWM + 50) {
        unfreezeIntegral();
    } else {
        freezeIntegral();
    }
}

int getPitchAngleInput(char source){
    if (source == PITCH_RC_SOURCE){
        return (int)((input_RC_PitchRate / ((float)HALF_PWM_RANGE / MAX_PITCH_ANGLE) ));
    }
    else if (source == PITCH_GS_SOURCE){
        return input_GS_Pitch;
    }
    else
        return 0;
}
int getPitchRateInput(char source){
    if (source == PITCH_RC_SOURCE){
        return input_RC_PitchRate;
    }
    else if (source == PITCH_GS_SOURCE){
        return input_GS_PitchRate;
    }
    else
        return 0;
}
int getRollAngleInput(char source){
    if (source == ROLL_RC_SOURCE){
        return (int)((input_RC_RollRate / ((float)HALF_PWM_RANGE / MAX_ROLL_ANGLE) ));
    }
    else if (source == ROLL_GS_SOURCE){
        return input_GS_Roll;
    }
    else{
        return 0;
    }
}
int getRollRateInput(char source){
    if (source == ROLL_RC_SOURCE){
        return input_RC_RollRate;
    }
    else if (source == ROLL_GS_SOURCE){
        return input_GS_RollRate;
    }
    else
        return 0;
}
int getYawAngleInput(char source){
    if (source == YAW_RC_SOURCE){
        return (int)((input_RC_YawRate / ((float)HALF_PWM_RANGE / MAX_ROLL_ANGLE) ));
    }
    else if (source == YAW_GS_SOURCE){
        return input_GS_Yaw;
    }
    else{
        return 0;
    }
}
int getYawRateInput(char source){
    if (source == YAW_RC_SOURCE){
        return input_RC_YawRate;
    }
    else if (source == YAW_GS_SOURCE){
        return input_GS_YawRate;
    }
    else
        return 0;
}
int getThrottleInput(char source){
    if (source == THROTTLE_RC_SOURCE){
        return input_RC_Throttle;
    }
    else if (source == THROTTLE_GS_SOURCE){
        return input_GS_Throttle;
    }
    else if (source == THROTTLE_AP_SOURCE){
//        return input_AP_Throttle;
        return 0;
    }
    else
        return 0;
}

int getFlapInput(char source){
    if (source == FLAP_RC_SOURCE){
        return input_RC_Flap;
    }
    else if (source == FLAP_GS_SOURCE){
        return input_GS_Flap;
    }
    else if (source == FLAP_AP_SOURCE){
//        return input_AP_Flap;
        return 0;
    }
    else
        return 0;
}

int getAltitudeInput(char source){
    if (source == ALTITUDE_GS_SOURCE){
        return input_GS_Altitude;
    }
    else if (source == ALTITUDE_AP_SOURCE){
        return input_AP_Altitude;
    }
    else
        return 0;
}

int getHeadingInput(char source){
    if (source == HEADING_GS_SOURCE){
        return input_GS_Heading;
    }
    else if (source == HEADING_AP_SOURCE){
        return input_AP_Heading;
    }
    else
        return 0;
}

void setKValues(int type,float* values){
    int Kchannel[7] = {YAW, PITCH, ROLL, HEADING, ALTITUDE, THROTTLE, FLAP};
    int i;
    for(i=0; i<7; i++){
       setGain(Kchannel[i],type,values[i]);
    }
}

void setGains(int channel, float* values){
    // values are found at index 1 to 3 in the data array
    setGain(channel,GAIN_KD,values[0]);
    setGain(channel,GAIN_KP,values[1]);
    setGain(channel,GAIN_KI,values[2]);
}

void imuCommunication(){
    /*****************************************************************************
     *****************************************************************************
                                IMU COMMUNICATION
     *****************************************************************************
     *****************************************************************************/
    VN100_SPI_GetRates(0, (float*) &imuData);

    //TODO: This is a reminder for me to figure out a more elegant way to fix improper derivative control (based on configuration of the sensor), adding this negative is a temporary fix. Some kind of calibration command or something.
    //DO NOT ADD NEGATIVES IN THE STATEMENTS BELOW. IT IS A GOOD WAY TO ROYALLY SCREW YOURSELF OVER LATER.
    //Outputs in order: Roll,Pitch,Yaw
    imu_RollRate = (imuData[IMU_ROLL_RATE]);
    imu_PitchRate = imuData[IMU_PITCH_RATE];
    imu_YawRate = imuData[IMU_YAW_RATE];
    VN100_SPI_GetYPR(0, &imuData[YAW], &imuData[PITCH], &imuData[ROLL]);
    imu_YawAngle = imuData[YAW];
    imu_PitchAngle = imuData[PITCH];
    imu_RollAngle = (imuData[ROLL]);
#if DEBUG
    // Rate - Radians, Angle - Degrees
//    char x[30];
//    sprintf(&x, "IMU Roll Rate: %f", imu_RollRate);
//    debug(&x);
//    sprintf(&x, "IMU Pitch Rate: %f", imu_PitchRate);
//    debug(&x);
//    sprintf(&x, "IMU Pitch Angle: %f", imu_PitchAngle);
//    debug(&x);
//    sprintf(&x, "IMU Roll Angle: %f", imu_RollAngle);
//    debug(&x);
#endif
}

int altitudeControl(int setpoint, int sensorAltitude){
    //Altitude
    ctrl_PitchAngle = controlSignalAltitude(setpoint, sensorAltitude) * ALTITUDE_TO_PITCH_DIRECTION; //TODO: Add -1 as a variable to flip directions
    if (ctrl_PitchAngle > MAX_PITCH_ANGLE)
        ctrl_PitchAngle = MAX_PITCH_ANGLE;
    if (ctrl_PitchAngle < -MAX_PITCH_ANGLE)
        ctrl_PitchAngle = -MAX_PITCH_ANGLE;
    return ctrl_PitchAngle;
}

int throttleControl(int setpoint, int sensor){
    //Throttle
    throttlePID = sp_ThrottleRate + controlSignalThrottle(setpoint, sensor);
    return throttlePID;
}

int flapControl(int setpoint, int sensor){
    //Flaps
    flapPID = sp_FlapRate + controlSignalFlap(setpoint, sensor);
    return flapPID;
}

//Equivalent to "Yaw Angle Control"
int headingControl(int setpoint, int sensor){
    //Heading
    while (setpoint > 360)
        setpoint -= 360;
    while (setpoint < 0)
        setpoint += 360;

    setHeadingSetpoint(setpoint);
    ctrl_Heading = controlSignalHeading(setpoint, sensor) * HEADING_TO_ROLL_DIRECTION;//gps_Satellites>=4?gps_Heading:(int)imu_YawAngle); //changed to monitor satellites, since we know these are good values while PositionFix might be corrupt...
    //Approximating Roll angle from Heading
    sp_RollAngle = ctrl_Heading;      //TODO: HOW IS HEADING HANDLED DIFFERENTLY BETWEEN QUADS AND PLANES

    if (sp_RollAngle > MAX_ROLL_ANGLE)
        sp_RollAngle = MAX_ROLL_ANGLE;
    if (sp_RollAngle < -MAX_ROLL_ANGLE)
        sp_RollAngle = -MAX_ROLL_ANGLE;
    return sp_RollAngle;
}


int rollAngleControl(int setpoint, int sensor){
    //Roll Angle
    sp_ComputedRollRate = controlSignalAngles(setpoint, sensor, ROLL, -(HALF_PWM_RANGE) / (MAX_ROLL_ANGLE));
    return sp_ComputedRollRate;
}

int pitchAngleControl(int setpoint, int sensor){
    //Pitch Angle
    sp_ComputedPitchRate = controlSignalAngles(setpoint, sensor, PITCH, -(HALF_PWM_RANGE) / (MAX_PITCH_ANGLE)); //Removed negative
    return sp_ComputedPitchRate;
}

int coordinatedTurn(float pitchRate, int rollAngle){
    //Feed forward Term when turning
    pitchRate += (int)(scaleFactor * abs(rollAngle)); //Linear Function
    return pitchRate;
}

int rollRateControl(float setpoint, float sensor){
    rollPID = controlSignal(setpoint/SERVO_SCALE_FACTOR, sensor, ROLL);
    return rollPID;
}
int pitchRateControl(float setpoint, float sensor){
    pitchPID = controlSignal(setpoint/SERVO_SCALE_FACTOR, sensor, PITCH);
    return pitchPID;
}
int yawRateControl(float setpoint, float sensor){
    yawPID = controlSignal(setpoint/SERVO_SCALE_FACTOR, sensor, YAW);
    return yawPID;
}

char getControlPermission(unsigned int controlMask, unsigned int expectedValue, char bitshift){
    int maskResult = (controlMask & controlLevel);
    return (maskResult >> bitshift) == expectedValue;
}

void readDatalink(void){

    struct command* cmd = popCommand();
    //TODO: Add rudimentary input validation
    if ( cmd ) {
        if (lastCommandSentCode[lastCommandCounter]/100 == cmd->cmd){
            lastCommandSentCode[lastCommandCounter]++;
        }
        else{
            lastCommandCounter++;
            lastCommandCounter %= COMMAND_HISTORY_SIZE;
            lastCommandSentCode[lastCommandCounter] = cmd->cmd * 100;
        }
        switch (cmd->cmd) {
            case DEBUG_TEST:             // Debugging command, writes to debug UART
#if DEBUG
                debug( (char*) cmd->data);
#endif
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

            case SET_FLAP_KD_GAIN:
                setGain(FLAP, GAIN_KD, *(float*)(&cmd->data));
                break;
            case SET_FLAP_KP_GAIN:
                setGain(FLAP, GAIN_KP, *(float*)(&cmd->data));
                break;
            case SET_FLAP_KI_GAIN:
                setGain(FLAP, GAIN_KI, *(float*)(&cmd->data));
                break;

            case SET_PATH_GAIN:
                amData.pathGain = *(float*)(&cmd->data);
                amData.command = PM_SET_PATH_GAIN;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SET_ORBIT_GAIN:
                amData.orbitGain = *(float*)(&cmd->data);
                amData.command = PM_SET_ORBIT_GAIN;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SHOW_GAIN:
                displayGain = *(char*)(&cmd->data);
                break;
            case SET_PITCH_RATE:
                input_GS_PitchRate = *(int*)(&cmd->data);
                break;
            case SET_ROLL_RATE:
                input_GS_RollRate = *(int*)(&cmd->data);
                break;
            case SET_YAW_RATE:
                input_GS_YawRate = *(int*)(&cmd->data);
                break;
            case SET_PITCH_ANGLE:
                input_GS_Pitch = *(int*)(&cmd->data);
                break;
            case SET_ROLL_ANGLE:
                input_GS_Roll = *(int*)(&cmd->data);
                break;
            case SET_YAW_ANGLE:
//                sp_YawAngle = *(int*)(&cmd->data);
                break;
            case SET_ALTITUDE:
                input_GS_Altitude = *(int*)(&cmd->data);
                break;
            case SET_HEADING:
                input_GS_Heading = *(int*)(&cmd->data);
                break;
            case SET_THROTTLE:
                input_GS_Throttle = *(int*)(&cmd->data);//(int)(((long int)(*(int*)(&cmd->data))) * MAX_PWM * 2 / 100) - MAX_PWM;
                break;
            case SET_FLAP:
                input_GS_Flap = *(int*)(&cmd->data);//(int)(((long int)(*(int*)(&cmd->data))) * MAX_PWM * 2 / 100) - MAX_PWM;
                break;
            case SET_AUTONOMOUS_LEVEL:
                controlLevel = *(int*)(&cmd->data);
                forceGainUpdate();
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
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case CLEAR_WAYPOINTS:
                amData.waypoint.id = (*(char *)(&cmd->data)); //Dummy Data
                amData.command = PM_CLEAR_WAYPOINTS;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case REMOVE_WAYPOINT:
                amData.waypoint.id = (*(char *)(&cmd->data));
                amData.command = PM_REMOVE_WAYPOINT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SET_TARGET_WAYPOINT:
                amData.waypoint.id = *(char *)(&cmd->data);
                amData.command = PM_SET_TARGET_WAYPOINT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case RETURN_HOME:
                amData.command = PM_RETURN_HOME;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case CANCEL_RETURN_HOME:
                amData.command = PM_CANCEL_RETURN_HOME;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SEND_HEARTBEAT:
                heartbeatTimer = getTime();
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
                    killPlane(TRUE);
                break;
            case UNKILL_PLANE:
                if (*(int*)(&cmd->data) == 1234)
                    killPlane(FALSE);
                break;
            case LOCK_GOPRO:
                    lockGoPro(*(int*)(&cmd->data));
                break;
            case ARM_VEHICLE:
                if (*(int*)(&cmd->data) == 1234)
                    armVehicle(500);
                break;
            case DEARM_VEHICLE:
                if (*(int*)(&cmd->data) == 1234)
                    dearmVehicle();
                break;
            case DROP_PROBE:
                dropProbe(*(char*)(&cmd->data));
                break;
            case RESET_PROBE:
                resetProbe(*(char*)(&cmd->data));
                break;
            case FOLLOW_PATH:
                amData.command = PM_FOLLOW_PATH;
                amData.followPath = *(char*)(&cmd->data);
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case EXIT_HOLD_ORBIT:
                amData.command = PM_EXIT_HOLD_ORBIT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SHOW_SCALED_PWM:
                if ((*(char*)(&cmd->data)) == 1){
                    show_scaled_pwm = 1;
                } else{
                    show_scaled_pwm = 0;
                }
                break;
            case NEW_WAYPOINT:
                amData.waypoint = *(WaypointWrapper*)(&cmd->data);
                amData.command = PM_NEW_WAYPOINT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case INSERT_WAYPOINT:
                amData.waypoint = *(WaypointWrapper*)(&cmd->data);
                amData.command = PM_INSERT_WAYPOINT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case UPDATE_WAYPOINT:
                amData.waypoint = *(WaypointWrapper*)(&cmd->data);
                amData.command = PM_UPDATE_WAYPOINT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SET_RETURN_HOME_COORDINATES:
                amData.waypoint = *(WaypointWrapper*)(&cmd->data);
                amData.command = PM_SET_RETURN_HOME_COORDINATES;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case TARE_IMU:
                adjustVNOrientationMatrix((float*)(&cmd->data));
                break;
            case SET_IMU:
                setVNOrientationMatrix((float*)(&cmd->data));
                break;
            case SET_KDVALUES:
                setKValues(GAIN_KD,(float*)(&cmd->data));
                break;
            case SET_KPVALUES:
                setKValues(GAIN_KP,(float*)(&cmd->data));
                break;
            case SET_KIVALUES:
                setKValues(GAIN_KI,(float*)(&cmd->data));
                break;
            case SET_GAINS:
            {
                char* channel = (char*) (&cmd->data);
                setGains(*channel,((float*)(&cmd->data)) + 1);
                break;
            }
            default:
                break;
        }
        destroyCommand( cmd );
    }

}
int writeDatalink(p_priority packet){
    struct telem_block* statusData = createTelemetryBlock(packet);

    //If Malloc fails, then quit...wait until there is memory available
    if (!statusData){return 0;}

    int* input;
    int* output; //Pointers used for RC channel inputs and outputs

    switch(packet){
        case PRIORITY0:
            statusData->data.p1_block.lat = getLatitude();
            statusData->data.p1_block.lon = getLongitude();
            statusData->data.p1_block.sysTime = getTime();
            statusData->data.p1_block.UTC = gps_Time;
            statusData->data.p1_block.pitch = getPitch();
            statusData->data.p1_block.roll = getRoll();
            statusData->data.p1_block.yaw = getYaw();
            statusData->data.p1_block.pitchRate = getPitchRate();
            statusData->data.p1_block.rollRate = getRollRate();
            statusData->data.p1_block.yawRate = getYawRate();
            statusData->data.p1_block.airspeed = airspeed;
            statusData->data.p1_block.alt = getAltitude();
            statusData->data.p1_block.gSpeed = gps_GroundSpeed;
            statusData->data.p1_block.heading = getHeading();
            statusData->data.p1_block.rollRateSetpoint = getRollRateSetpoint();
            statusData->data.p1_block.rollSetpoint = getRollAngleSetpoint();
            statusData->data.p1_block.pitchRateSetpoint = getPitchRateSetpoint();
            statusData->data.p1_block.pitchSetpoint = getPitchAngleSetpoint();
            statusData->data.p1_block.throttleSetpoint = getThrottleSetpoint();
            break;
        case PRIORITY1:
            statusData->data.p2_block.rollKD = getGain(ROLL,GAIN_KD);
            statusData->data.p2_block.rollKP = getGain(ROLL,GAIN_KP);
            statusData->data.p2_block.pitchKD = getGain(PITCH,GAIN_KD);
            statusData->data.p2_block.pitchKP = getGain(PITCH,GAIN_KP);
            statusData->data.p2_block.yawKD = getGain(YAW,GAIN_KD);
            statusData->data.p2_block.yawKP = getGain(YAW,GAIN_KP);
            statusData->data.p2_block.lastCommandsSent[0] = lastCommandSentCode[lastCommandCounter];
            statusData->data.p2_block.lastCommandsSent[1] = lastCommandSentCode[(lastCommandCounter + (COMMAND_HISTORY_SIZE - 1))%COMMAND_HISTORY_SIZE];
            statusData->data.p2_block.lastCommandsSent[2] = lastCommandSentCode[(lastCommandCounter + (COMMAND_HISTORY_SIZE - 2))%COMMAND_HISTORY_SIZE];
            statusData->data.p2_block.lastCommandsSent[3] = lastCommandSentCode[(lastCommandCounter + (COMMAND_HISTORY_SIZE - 3))%COMMAND_HISTORY_SIZE];
            statusData->data.p2_block.batteryLevel1 = batteryLevel1;
            statusData->data.p2_block.batteryLevel2 = batteryLevel2;
//            debug("SW3");
            if (show_scaled_pwm){
                input = getPWMArray(getTime());
            } else {
                input = (int*)getICValues(getTime());
            }
            statusData->data.p2_block.ch1In = input[0];
            statusData->data.p2_block.ch2In = input[1];
            statusData->data.p2_block.ch3In = input[2];
            statusData->data.p2_block.ch4In = input[3];
            statusData->data.p2_block.ch5In = input[4];
            statusData->data.p2_block.ch6In = input[5];
            statusData->data.p2_block.ch7In = input[6];
            statusData->data.p2_block.ch8In = input[7];
            output = getPWMOutputs();
            statusData->data.p2_block.ch1Out = output[0];
            statusData->data.p2_block.ch2Out = output[1];
            statusData->data.p2_block.ch3Out = output[2];
            statusData->data.p2_block.ch4Out = output[3];
            statusData->data.p2_block.ch5Out = output[4];
            statusData->data.p2_block.ch6Out = output[5];
            statusData->data.p2_block.ch7Out = output[6];
            statusData->data.p2_block.ch8Out = output[7];
//            debug("SW4");
            statusData->data.p2_block.yawRateSetpoint = getYawRateSetpoint();
            statusData->data.p2_block.headingSetpoint = getHeadingSetpoint();
            statusData->data.p2_block.altitudeSetpoint = getAltitudeSetpoint();
            statusData->data.p2_block.flapSetpoint = getFlapSetpoint();
            statusData->data.p2_block.cameraStatus = cameraCounter;
            statusData->data.p2_block.wirelessConnection = ((input[5] < 180) << 1) + (input[7] > 0);//+ RSSI;
            statusData->data.p2_block.autopilotActive = getProgramStatus();
            //statusData->data.p2_block.sensorStat = getSensorStatus(0) + (getSensorStatus(1) << 2);
            statusData->data.p2_block.gpsStatus = gps_Satellites + (gps_PositionFix << 4);
            statusData->data.p2_block.pathChecksum = waypointChecksum;
            statusData->data.p2_block.numWaypoints = waypointCount;
            statusData->data.p2_block.waypointIndex = waypointIndex;
            statusData->data.p2_block.pathFollowing = pathFollowing; //True or false
            break;
        case PRIORITY2:
            statusData->data.p3_block.rollKI = getGain(ROLL,GAIN_KI);
            statusData->data.p3_block.pitchKI = getGain(PITCH,GAIN_KI);
            statusData->data.p3_block.yawKI = getGain(YAW, GAIN_KI);
            statusData->data.p3_block.headingKD = getGain(HEADING, GAIN_KD);
            statusData->data.p3_block.headingKP = getGain(HEADING, GAIN_KP);
            statusData->data.p3_block.headingKI = getGain(HEADING, GAIN_KI);
            statusData->data.p3_block.altitudeKD = getGain(ALTITUDE, GAIN_KD);
            statusData->data.p3_block.altitudeKP = getGain(ALTITUDE, GAIN_KP);
            statusData->data.p3_block.altitudeKI = getGain(ALTITUDE, GAIN_KI);
            statusData->data.p3_block.throttleKD = getGain(THROTTLE, GAIN_KD);
            statusData->data.p3_block.throttleKP = getGain(THROTTLE, GAIN_KP);
            statusData->data.p3_block.throttleKI = getGain(THROTTLE, GAIN_KI);
            statusData->data.p3_block.flapKD = getGain(FLAP, GAIN_KD);
            statusData->data.p3_block.flapKP = getGain(FLAP, GAIN_KP);
            statusData->data.p3_block.flapKI = getGain(FLAP, GAIN_KI);
            statusData->data.p3_block.pathGain = pmPathGain;
            statusData->data.p3_block.orbitGain = pmOrbitGain;
            statusData->data.p3_block.autonomousLevel = controlLevel;
            statusData->data.p3_block.startupErrorCodes = getStartupErrorCodes();
            statusData->data.p3_block.startupSettings = DEBUG + (MULTIROTOR << 1); //TODO: put this in the startuperrorCode file
            statusData->data.p3_block.probeStatus = getProbeStatus();
            break;

        default:
            break;
    }

    if (BLOCKING_MODE) {
        sendTelemetryBlock(statusData);
        destroyTelemetryBlock(statusData);
    } else {
        return pushOutboundTelemetryQueue(statusData);
    }

    return 0;

}

void checkUHFStatus(){
    unsigned long int time = getTime();

    if (getPWMInputStatus() == PWM_STATUS_UHF_LOST){
        if (UHFTimer == 0){ //0 indicates that this is the first time we lost UHF
            UHFTimer = time;
            setProgramStatus(KILL_MODE_WARNING);
        } else { //otherwise check if we're over the threshold
            if (time - UHFTimer > UHF_KILL_TIMEOUT){
                killPlane(TRUE);
            }
        }
    } else {
        UHFTimer = 0; //set it back to 0 otherwise to reset state
    }
}

void checkHeartbeat(){
    if (getTime() - heartbeatTimer > HEARTBEAT_TIMEOUT){
        setProgramStatus(KILL_MODE_WARNING);
        amData.command = PM_RETURN_HOME;
        amData.checkbyteDMA = generateAMDataDMACheckbyte();
        amData.checksum = generateAMDataChecksum(&amData);
    }
    else if (getTime() - heartbeatTimer > HEARTBEAT_KILL_TIMEOUT){
        killPlane(TRUE);
    }
}

void checkGPS(){
//    if (gps_PositionFix == 0){
//        setProgramStatus(KILL_MODE_WARNING);
//        if (getTime() - gpsTimer > GPS_TIMEOUT){
//            killPlane(TRUE);
//        }
//    }
//    else{
//        gpsTimer = getTime();
//    }
}


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
