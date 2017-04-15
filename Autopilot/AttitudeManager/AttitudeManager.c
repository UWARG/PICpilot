/*
 * File:   AttitudeManager.c
 * Author: Mitch
 *
 * Created on June 15, 2013, 3:40 PM
 */

//Include Header Files
#include "AttitudeManager.h"
#include "delay.h"
#include "VN100.h"
#include "InputCapture.h"
#include "OutputCompare.h"
#include "PWM.h"
#include "AttitudeManager.h"
#include "Network/Commands.h"
#include "StartupErrorCodes.h"
#include "main.h"
#include "../Common/Interfaces/SPI.h"
#include "Network/Datalink.h"
#include "ProgramStatus.h"
#include "Drivers/Radio.h"
#include "Peripherals/UHF.h"
#include <string.h>

int input_RC_Flap; // Flaps need to finish being refactored.
int input_GS_Flap;

extern PMData pmData;
extern AMData amData;
extern char DMADataAvailable;

// Control level bit masks (indexed by CtrlType enum)
static const uint16_t ctrl_mask[16] = {
            0b00000001, // Pitch Rate(0) or Pitch Angles(1)
            0b00000010, // Pitch from Controller(0) or Ground Station(1)
            0b00000100, // Roll Rates(0) or Roll Angles(1)
            0b00001000, // Roll from Controller(0) or Ground Station(1)
            0b00110000, // Throttle from Controller(0) or Ground Station(1) or Autopilot(2)(Controlled by the GroundSpeed/Altitude).
            0b00000000, // Empty. Throttle is 2 bits, so index 5 is unused.
            0b01000000, // Altitude from Ground Station(0) or Autopilot(1)
            0b10000000, // Altitude Off(0) or On(1)
    0b0000000100000000, // Heading from Ground Station(0) or Autopilot(1)
    0b0000001000000000, // Heading Off(0) or On(1)
    0b0000110000000000  // Flaps from Controller(0) or Ground Station(1) or Autopilot(2) // TODO:remove
};

long int heartbeatTimer = 0;
long int UHFTimer = 0;
long int gpsTimer = 0;

// Setpoints
int sp_Throttle = MIN_PWM;

int sp_RollRate = 0; // Degrees per second
int sp_PitchRate = 0;
int sp_YawRate = 0;

int sp_RollAngle = 0; // degrees
int sp_PitchAngle = 0;

int sp_Heading = 0;
int sp_Altitude = 0;
float sp_GroundSpeed = 0;

bool limitSetpoint = true;

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

// System outputs (get from IMU)
float imu_RollRate = 0;
float imu_PitchRate = 0;
float imu_YawRate = 0;

//IMU integration outputs
float imu_RollAngle = 0;
float imu_PitchAngle = 0;
float imu_YawAngle = 0;

//RC Input Signals (Input Capture Values)
int input_RC_Throttle = MIN_PWM;
int input_RC_RollRate = 0;
int input_RC_PitchRate = 0;
int input_RC_YawRate = 0;
int input_RC_Aux1 = 0; //0=Roll, 1= Pitch, 2=Yaw
int input_RC_Aux2 = 0; //0 = Saved Value, 1 = Edit Mode
int input_RC_Switch1 = 0;

//Ground Station Input Signals
int input_GS_Roll = 0;
int input_GS_Pitch = 0;
int input_GS_Throttle = 0;
int input_GS_Yaw = 0;
int input_GS_RollRate = 0;
int input_GS_PitchRate = 0;
int input_GS_YawRate = 0;
int input_GS_Altitude = 0;
int input_GS_Heading = 0;

int input_AP_Altitude = 0;
int input_AP_Heading = 0;

float scaleFactor = 5; // Roll angle -> pitch rate scaling (for fixed-wing turns) 

char displayGain = 0;
int controlLevel = 0;
int lastCommandSentCode[COMMAND_HISTORY_SIZE];
int lastCommandCounter = 0;

char show_scaled_pwm = 1;

void attitudeInit() {
    setProgramStatus(INITIALIZATION);
    //Initialize Timer
    initTimer4();
    
    amData.checkbyteDMA = generateAMDataDMACheckbyte();
    init_DMA0(1);
    init_DMA1(1);
    initSPI(IC_DMA_PORT, 0, SPI_MODE1, SPI_BYTE, SPI_SLAVE);


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
    float filterVariance[10] = {1e-9, 1e-9, 1e-9, 1e-9, 1, 1, 1, 1e-4, 1e-4, 1e-4}; //  {1e-10, 1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
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
    
    orientationInit();
    initDatalink();
    setSensorStatus(XBEE, SENSOR_INITIALIZED & TRUE);
    initialization();
    setProgramStatus(MAIN_EXECUTION);
}


char checkDMA(){
    //Transfer data from PATHMANAGER CHIP
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
        if (gps_Altitude == pmData.altitude && gps_Heading == pmData.heading && gps_GroundSpeed == pmData.speed / 3.6f && gps_Latitude == pmData.latitude && gps_Longitude == pmData.longitude){
            return FALSE;
        }
        
        debug("New DMA data!");

        gps_Heading = pmData.heading;
        gps_GroundSpeed = pmData.speed / 3.6f; //Convert from km/h to m/s
        gps_Longitude = pmData.longitude;
        gps_Latitude = pmData.latitude;
        gps_Altitude = pmData.altitude;

        if (gps_PositionFix){
            input_AP_Heading = pmData.sp_Heading;
        }
        return TRUE;
    }
    else{
        debug("DMA Reset occured");
//        INTERCOM_2 = 1;
        DMA0CONbits.CHEN = 0; //Disable DMA0 channel
        DMA1CONbits.CHEN = 0; //Disable DMA1 channel
        SPI1STATbits.SPIEN = 0; //Disable SPI1

        while(SPI1STATbits.SPIRBF) { //Clear SPI1
            SPI1BUF;
        }
//        INTERCOM_2 = 0;
//        while(INTERCOM_4);

        init_DMA0(1);
        init_DMA1(1);
        initSPI(IC_DMA_PORT, 0, SPI_MODE1, SPI_BYTE, SPI_SLAVE);

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
    return sp_Throttle;
}
int getAltitudeSetpoint(){
    return sp_Altitude;
}
int getHeadingSetpoint(){
    return sp_Heading;
}

void setPitchAngleSetpoint(int setpoint){
    if (limitSetpoint) {
        constrain(&setpoint, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
    }
    sp_PitchAngle = setpoint;
}
void setRollAngleSetpoint(int setpoint){
    if (limitSetpoint) {
        constrain(&setpoint, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
    }
    sp_RollAngle = setpoint;
}
void setPitchRateSetpoint(int setpoint){
    if (limitSetpoint) {
        constrain(&setpoint, -MAX_PITCH_RATE, MAX_PITCH_RATE);
    }
    sp_PitchRate = setpoint;
}
void setRollRateSetpoint(int setpoint){
    if (limitSetpoint) {
        constrain(&setpoint, -MAX_ROLL_RATE, MAX_ROLL_RATE);
    }
    sp_RollRate = setpoint;
}
void setYawRateSetpoint(int setpoint){
    if (limitSetpoint) {
        constrain(&setpoint, -MAX_YAW_RATE, MAX_YAW_RATE);
    }
    sp_YawRate = setpoint;
}
void setThrottleSetpoint(int setpoint){
    sp_Throttle = setpoint;
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

}

int getPitchAngleInput(char source){
    if (source == RC_SOURCE){
        return (int)(input_RC_PitchRate / (HALF_PWM_RANGE / MAX_PITCH_ANGLE));
    }
    else if (source == GS_SOURCE){
        return input_GS_Pitch;
    }
    else
        return 0;
}
int getPitchRateInput(char source){
    if (source == RC_SOURCE){
        return (int)(input_RC_PitchRate / (HALF_PWM_RANGE / MAX_PITCH_RATE));
    }
    else if (source == GS_SOURCE){
        return input_GS_PitchRate;
    }
    else
        return 0;
}
int getRollAngleInput(char source){
    if (source == RC_SOURCE){
        return (int)(input_RC_RollRate / (HALF_PWM_RANGE / MAX_ROLL_ANGLE));
    }
    else if (source == GS_SOURCE){
        return input_GS_Roll;
    }
    else{
        return 0;
    }
}
int getRollRateInput(char source){
    if (source == RC_SOURCE){
        return (int)(input_RC_RollRate / (HALF_PWM_RANGE / MAX_ROLL_RATE));
    }
    else if (source == GS_SOURCE){
        return input_GS_RollRate;
    }
    else
        return 0;
}

int getYawRateInput(char source){
    if (source == RC_SOURCE){
        return (int)(input_RC_YawRate / (HALF_PWM_RANGE / MAX_YAW_RATE));
    }
    else if (source == GS_SOURCE){
        return input_GS_YawRate;
    }
    else
        return 0;
}
int getThrottleInput(char source){
    if (source == RC_SOURCE){
        return input_RC_Throttle;
    }
    else if (source == GS_SOURCE){
        return input_GS_Throttle;
    }
    else if (source == AP_SOURCE){
//        return input_AP_Throttle;
        return MIN_PWM;
    }
    else
        return 0;
}

int getFlapInput(char source){
    if (source == RC_SOURCE){
        return input_RC_Flap;
    }
    else if (source == GS_SOURCE){
        return input_GS_Flap;
    }
    else if (source == AP_SOURCE){
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
    int i = 0;
    for(; i<CONTROL_CHANNELS; i++){
       setGain(i, type, values[i]);
    }
}

void setGains(int channel, float* values){
    // values are found at index 1 to 3 in the data array
    setGain(channel,KP,values[0]);
    setGain(channel,KI,values[1]);
    setGain(channel,KD,values[2]);
}

void imuCommunication(){
    /*****************************************************************************
     *****************************************************************************
                                IMU COMMUNICATION
     *****************************************************************************
     *****************************************************************************/
    float imuData[3];

    VN100_SPI_GetYPR(0, &imu_YawAngle, &imu_PitchAngle, &imu_RollAngle);
    VN100_SPI_GetRates(0, imuData);

    /* TODO: This is a reminder for me to figure out a more elegant way to fix improper derivative control
     * (based on configuration of the sensor), adding this negative is a temporary fix. Some kind of calibration command or something.
     */
    //Outputs in order: Roll,Pitch,Yaw
    imu_RollRate = rad2deg(imuData[IMU_ROLL_RATE]);
    imu_PitchRate = rad2deg(imuData[IMU_PITCH_RATE]);
    imu_YawRate = rad2deg(imuData[IMU_YAW_RATE]);
}

int coordinatedTurn(float pitchRate, int rollAngle){
    //Feed forward Term when turning
    pitchRate += (int)(scaleFactor * abs(rollAngle)); //Linear Function
    return pitchRate;
}

// Type is both bit shift value and index of bit mask array
uint8_t getControlValue(CtrlType type) {
    return (controlLevel & ctrl_mask[type]) >> type;
}

void readDatalink(void){
    struct DatalinkCommand* cmd = popDatalinkCommand();
    
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
                setGain(PITCH_RATE, KD, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_ROLL_KD_GAIN:
                setGain(ROLL_RATE, KD, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_YAW_KD_GAIN:
                setGain(YAW_RATE, KD, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_PITCH_KP_GAIN:
                setGain(PITCH_RATE, KP, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_ROLL_KP_GAIN:
                setGain(ROLL_RATE, KP, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_YAW_KP_GAIN:
                setGain(YAW_RATE, KP, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_PITCH_KI_GAIN:
                setGain(PITCH_RATE, KI, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_ROLL_KI_GAIN:
                setGain(ROLL_RATE, KI, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_YAW_KI_GAIN:
                setGain(YAW_RATE, KI, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_HEADING_KD_GAIN:
                setGain(HEADING, KD, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_HEADING_KP_GAIN:
                setGain(HEADING, KP, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_HEADING_KI_GAIN:
                setGain(HEADING, KI, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_ALTITUDE_KD_GAIN:
                setGain(ALTITUDE, KD, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_ALTITUDE_KP_GAIN:
                setGain(ALTITUDE, KP, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_ALTITUDE_KI_GAIN:
                setGain(ALTITUDE, KI, CMD_TO_FLOAT(cmd->data));
                break;
            case SET_THROTTLE_KD_GAIN:
                //setGain(THROTTLE, KD, *(float*)(&cmd->data));
                break;
            case SET_THROTTLE_KP_GAIN:
                //setGain(THROTTLE, KP, *(float*)(&cmd->data));
                break;
            case SET_THROTTLE_KI_GAIN:
                //setGain(THROTTLE, KI, *(float*)(&cmd->data));
                break;

            case SET_PATH_GAIN:
                amData.pathGain = CMD_TO_FLOAT(cmd->data);
                amData.command = PM_SET_PATH_GAIN;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SET_ORBIT_GAIN:
                amData.orbitGain = CMD_TO_FLOAT(cmd->data);
                amData.command = PM_SET_ORBIT_GAIN;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SHOW_GAIN:
                displayGain = *cmd->data;
                break;
            case SET_PITCH_RATE:
                input_GS_PitchRate = CMD_TO_INT(cmd->data);
                break;
            case SET_ROLL_RATE:
                input_GS_RollRate = CMD_TO_INT(cmd->data);
                break;
            case SET_YAW_RATE:
                input_GS_YawRate = CMD_TO_INT(cmd->data);
                break;
            case SET_PITCH_ANGLE:
                input_GS_Pitch = CMD_TO_INT(cmd->data);
                break;
            case SET_ROLL_ANGLE:
                input_GS_Roll = CMD_TO_INT(cmd->data);
                break;
            case SET_YAW_ANGLE:
//                sp_YawAngle = *(int*)(&cmd->data);
                break;
            case SET_ALTITUDE:
                input_GS_Altitude = CMD_TO_INT(cmd->data);
                break;
            case SET_HEADING:
                input_GS_Heading = CMD_TO_INT(cmd->data);
                break;
            case SET_THROTTLE:
                input_GS_Throttle = CMD_TO_INT(cmd->data);
                break;
            case SET_FLAP:
                input_GS_Flap = CMD_TO_INT(cmd->data);//(int)(((long int)(*(int*)(&cmd->data))) * MAX_PWM * 2 / 100) - MAX_PWM;
                break;
            case SET_AUTONOMOUS_LEVEL:
                controlLevel = CMD_TO_INT(cmd->data);
                forceGainUpdate();
                break;
            case SET_ANGULAR_WALK_VARIANCE:
                setAngularWalkVariance(CMD_TO_FLOAT(cmd->data));
                break;
            case SET_GYRO_VARIANCE:
                setGyroVariance(CMD_TO_FLOAT(cmd->data));
                break;
            case SET_MAGNETIC_VARIANCE:
                setMagneticVariance(CMD_TO_FLOAT(cmd->data));
                break;
            case SET_ACCEL_VARIANCE:
                setAccelVariance(CMD_TO_FLOAT(cmd->data));
                break;
            case SET_SCALE_FACTOR:
                scaleFactor = CMD_TO_FLOAT(cmd->data);
                break;
            case CALIBRATE_ALTIMETER:
                amData.calibrationHeight = CMD_TO_FLOAT(cmd->data);
                amData.command = PM_CALIBRATE_ALTIMETER;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case CLEAR_WAYPOINTS:
                amData.waypoint.id = *cmd->data; //Dummy Data
                amData.command = PM_CLEAR_WAYPOINTS;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case REMOVE_WAYPOINT:
                amData.waypoint.id = *cmd->data;
                amData.command = PM_REMOVE_WAYPOINT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SET_TARGET_WAYPOINT:
                amData.waypoint.id = *cmd->data;
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
                queueRadioStatusPacket();
                break;
            case KILL_PLANE:
                if (CMD_TO_INT(cmd->data) == 1234)
                    killPlane(TRUE);
                break;
            case UNKILL_PLANE:
                if (CMD_TO_INT(cmd->data) == 1234)
                    killPlane(FALSE);
                break;
            case ARM_VEHICLE:
                if (CMD_TO_INT(cmd->data) == 1234)
                    armVehicle(500);
                break;
            case DEARM_VEHICLE:
                if (CMD_TO_INT(cmd->data) == 1234)
                    dearmVehicle();
                break;
            case FOLLOW_PATH:
                amData.command = PM_FOLLOW_PATH;
                amData.followPath = *cmd->data;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case EXIT_HOLD_ORBIT:
                amData.command = PM_EXIT_HOLD_ORBIT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SHOW_SCALED_PWM:
                if (*cmd->data == 1){
                    show_scaled_pwm = 1;
                } else{
                    show_scaled_pwm = 0;
                }
                break;
            case REMOVE_LIMITS:
                limitSetpoint = *(bool*)cmd->data;
            case NEW_WAYPOINT:
                amData.waypoint = CMD_TO_TYPE(cmd->data, WaypointWrapper);
                amData.command = PM_NEW_WAYPOINT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case INSERT_WAYPOINT:
                amData.waypoint = CMD_TO_TYPE(cmd->data, WaypointWrapper);
                amData.command = PM_INSERT_WAYPOINT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case UPDATE_WAYPOINT:
                amData.waypoint = CMD_TO_TYPE(cmd->data, WaypointWrapper);
                amData.command = PM_UPDATE_WAYPOINT;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case SET_RETURN_HOME_COORDINATES:
                amData.waypoint = CMD_TO_TYPE(cmd->data, WaypointWrapper);
                amData.command = PM_SET_RETURN_HOME_COORDINATES;
                amData.checkbyteDMA = generateAMDataDMACheckbyte();
                amData.checksum = generateAMDataChecksum(&amData);
                break;
            case TARE_IMU:
                adjustVNOrientationMatrix(CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_IMU:
                setVNOrientationMatrix(CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_KDVALUES:
                setKValues(KD, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_KPVALUES:
                setKValues(KP, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_KIVALUES:
                setKValues(KI, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_GAINS:
            {
                char channel = cmd->data[0];
                //We're making the rest of the data word aligned here, which is required for casting it to floats
                memcpy(cmd->data, cmd->data + 1, cmd->data_length - 1);
                setGains(channel, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            }
            default:
                break;
        }
       freeDatalinkCommand( cmd );
    }
}

bool writeDatalink(p_priority packet){
    static TelemetryBlock statusData;

    int* input;
    int* output; //Pointers used for RC channel inputs and outputs
    statusData.type = packet;
    
    switch(packet){
        case PRIORITY0:
            statusData.data.p1_block.lat = getLatitude();
            statusData.data.p1_block.lon = getLongitude();
            statusData.data.p1_block.sysTime = getTime();
            statusData.data.p1_block.UTC = gps_Time;
            statusData.data.p1_block.pitch = getPitch();
            statusData.data.p1_block.roll = getRoll();
            statusData.data.p1_block.yaw = getYaw();
            statusData.data.p1_block.pitchRate = getPitchRate();
            statusData.data.p1_block.rollRate = getRollRate();
            statusData.data.p1_block.yawRate = getYawRate();
            statusData.data.p1_block.airspeed = airspeed;
            statusData.data.p1_block.alt = getAltitude();
            statusData.data.p1_block.gSpeed = gps_GroundSpeed;
            statusData.data.p1_block.heading = getHeading();
            statusData.data.p1_block.rollRateSetpoint = getRollRateSetpoint();
            statusData.data.p1_block.rollSetpoint = getRollAngleSetpoint();
            statusData.data.p1_block.pitchRateSetpoint = getPitchRateSetpoint();
            statusData.data.p1_block.pitchSetpoint = getPitchAngleSetpoint();
            statusData.data.p1_block.throttleSetpoint = getThrottleSetpoint();
            break;
        case PRIORITY1:
            statusData.data.p2_block.rollKD = getGain(ROLL_RATE,KD);
            statusData.data.p2_block.rollKP = getGain(ROLL_RATE,KP);
            statusData.data.p2_block.pitchKD = getGain(PITCH_RATE,KD);
            statusData.data.p2_block.pitchKP = getGain(PITCH_RATE,KP);
            statusData.data.p2_block.yawKD = getGain(YAW_RATE,KD);
            statusData.data.p2_block.yawKP = getGain(YAW_RATE,KP);
            statusData.data.p2_block.lastCommandsSent[0] = lastCommandSentCode[lastCommandCounter];
            statusData.data.p2_block.lastCommandsSent[1] = lastCommandSentCode[(lastCommandCounter + (COMMAND_HISTORY_SIZE - 1))%COMMAND_HISTORY_SIZE];
            statusData.data.p2_block.lastCommandsSent[2] = lastCommandSentCode[(lastCommandCounter + (COMMAND_HISTORY_SIZE - 2))%COMMAND_HISTORY_SIZE];
            statusData.data.p2_block.lastCommandsSent[3] = lastCommandSentCode[(lastCommandCounter + (COMMAND_HISTORY_SIZE - 3))%COMMAND_HISTORY_SIZE];
            statusData.data.p2_block.batteryLevel1 = 50;//batteryLevel1;
            statusData.data.p2_block.batteryLevel2 = 50;//batteryLevel2;
//            debug("SW3");
            if (show_scaled_pwm){
                input = getPWMArray(getTime());
            } else {
                input = (int*)getICValues(getTime());
            }
            statusData.data.p2_block.ch1In = input[0];
            statusData.data.p2_block.ch2In = input[1];
            statusData.data.p2_block.ch3In = input[2];
            statusData.data.p2_block.ch4In = input[3];
            statusData.data.p2_block.ch5In = input[4];
            statusData.data.p2_block.ch6In = input[5];
            statusData.data.p2_block.ch7In = input[6];
            statusData.data.p2_block.ch8In = input[7];
            output = getPWMOutputs();
            statusData.data.p2_block.ch1Out = output[0];
            statusData.data.p2_block.ch2Out = output[1];
            statusData.data.p2_block.ch3Out = output[2];
            statusData.data.p2_block.ch4Out = output[3];
            statusData.data.p2_block.ch5Out = output[4];
            statusData.data.p2_block.ch6Out = output[5];
            statusData.data.p2_block.ch7Out = output[6];
            statusData.data.p2_block.ch8Out = output[7];
//            debug("SW4");
            statusData.data.p2_block.yawRateSetpoint = getYawRateSetpoint();
            statusData.data.p2_block.headingSetpoint = getHeadingSetpoint();
            statusData.data.p2_block.altitudeSetpoint = getAltitudeSetpoint();
            statusData.data.p2_block.flapSetpoint = 0;
            statusData.data.p2_block.wirelessConnection = ((input[5] < 180) << 1) + (input[7] > 0);//+ RSSI;
            statusData.data.p2_block.autopilotActive = getProgramStatus();
            statusData.data.p2_block.gpsStatus = gps_Satellites + (gps_PositionFix << 4);
            statusData.data.p2_block.pathChecksum = waypointChecksum;
            statusData.data.p2_block.numWaypoints = waypointCount;
            statusData.data.p2_block.waypointIndex = waypointIndex;
            statusData.data.p2_block.pathFollowing = pathFollowing; //True or false
            break;
        case PRIORITY2:
            statusData.data.p3_block.rollKI = getGain(ROLL_RATE,KI);
            statusData.data.p3_block.pitchKI = getGain(PITCH_RATE,KI);
            statusData.data.p3_block.yawKI = getGain(YAW_RATE, KI);
            statusData.data.p3_block.headingKD = getGain(HEADING, KD);
            statusData.data.p3_block.headingKP = getGain(HEADING, KP);
            statusData.data.p3_block.headingKI = getGain(HEADING, KI);
            statusData.data.p3_block.altitudeKD = getGain(ALTITUDE, KD);
            statusData.data.p3_block.altitudeKP = getGain(ALTITUDE, KP);
            statusData.data.p3_block.altitudeKI = getGain(ALTITUDE, KI);
            statusData.data.p3_block.throttleKD = 0;//getGain(THROTTLE, KD);
            statusData.data.p3_block.throttleKP = 0;//getGain(THROTTLE, KP);
            statusData.data.p3_block.throttleKI = 0;//getGain(THROTTLE, KI);
            statusData.data.p3_block.flapKD = 0;//getGain(FLAP, KD);
            statusData.data.p3_block.flapKP = 0;//getGain(FLAP, KP);
            statusData.data.p3_block.flapKI = 0;//getGain(FLAP, KI);
            statusData.data.p3_block.pathGain = pmPathGain;
            statusData.data.p3_block.orbitGain = pmOrbitGain;
            statusData.data.p3_block.autonomousLevel = controlLevel;
            statusData.data.p3_block.startupErrorCodes = getStartupErrorCodes();
            statusData.data.p3_block.startupSettings = DEBUG + (VEHICLE_TYPE << 1); //TODO: put this in the startuperrorCode file
            statusData.data.p3_block.ul_rssi = getRadioRSSI();
            statusData.data.p3_block.ul_receive_errors = getRadioReceiveErrors();
            statusData.data.p3_block.dl_transmission_errors = getRadioTransmissionErrors();
            statusData.data.p3_block.uhf_link_quality = getUHFLinkQuality(getTime());
            statusData.data.p3_block.uhf_rssi = getUHFRSSI(getTime());
            break;

        default:
            break;
    }
    return queueTelemetryBlock(&statusData);
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
