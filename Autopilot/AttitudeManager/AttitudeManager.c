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
#include "Network/Commands.h"
#include "StartupErrorCodes.h"
#include "main.h"
#include "../Common/Interfaces/SPI.h"
#include "Network/Datalink.h"
#include "ProgramStatus.h"
#include "Drivers/Radio.h"
#include "Peripherals/UHF.h"
#include "../Common/Interfaces/InterchipDMA.h"
#include "../Common/Clock/Timer.h"
#include "../Common/Utilities/LED.h"
#include <string.h>

int input_RC_Flap; // Flaps need to finish being refactored.
int input_GS_Flap;

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


static uint16_t pm_interchip_error_count = 0;
static uint16_t gps_communication_error_count = 0;
static bool show_gains = false;
static bool show_scaled_pwm = true;

void attitudeInit() {
    setProgramStatus(INITIALIZATION);
    //Initialize Timer
    initTimer4();
    
    initLED(1);
    
    initSPI(IC_DMA_PORT, 0, SPI_MODE1, SPI_BYTE, SPI_SLAVE);
    initInterchip(DMA_CHIP_ID_ATTITUDE_MANAGER);
    
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
    
    orientationInit();
    initDatalink();
    setSensorStatus(XBEE, SENSOR_INITIALIZED & TRUE);
    initialization();
    setProgramStatus(MAIN_EXECUTION);
}

char checkDMA(){
        gps_Time = interchip_receive_buffer.pm_data.time;
        input_AP_Altitude = interchip_receive_buffer.pm_data.sp_Altitude;
        gps_Satellites = interchip_receive_buffer.pm_data.satellites;
        gps_PositionFix = interchip_receive_buffer.pm_data.positionFix;
        waypointIndex = interchip_receive_buffer.pm_data.targetWaypoint;
        batteryLevel1 = interchip_receive_buffer.pm_data.batteryLevel1;
        batteryLevel2 = interchip_receive_buffer.pm_data.batteryLevel2;
        waypointCount = interchip_receive_buffer.pm_data.waypointCount;
        waypointChecksum = interchip_receive_buffer.pm_data.waypointChecksum;
        pathFollowing = interchip_receive_buffer.pm_data.pathFollowing;
        airspeed = interchip_receive_buffer.pm_data.airspeed;
        pmOrbitGain = interchip_receive_buffer.pm_data.pmOrbitGain;
        pmPathGain = interchip_receive_buffer.pm_data.pmPathGain;
        
         //Check if this data is new and requires action or if it is old and redundant
        if (gps_Altitude == interchip_receive_buffer.pm_data.altitude 
                && gps_Heading == interchip_receive_buffer.pm_data.heading 
                && gps_GroundSpeed == interchip_receive_buffer.pm_data.speed / 3.6f 
                && gps_Latitude == interchip_receive_buffer.pm_data.latitude 
                && gps_Longitude == interchip_receive_buffer.pm_data.longitude){    
            return FALSE;		
        }

        gps_Heading = interchip_receive_buffer.pm_data.heading;
        gps_GroundSpeed = interchip_receive_buffer.pm_data.speed / 3.6f; //Convert from km/h to m/s
        gps_Longitude = interchip_receive_buffer.pm_data.longitude;
        gps_Latitude = interchip_receive_buffer.pm_data.latitude;
        gps_Altitude = interchip_receive_buffer.pm_data.altitude;
        pm_interchip_error_count = interchip_receive_buffer.pm_data.interchip_error_count;
        gps_communication_error_count = interchip_receive_buffer.pm_data.gps_communication_error_count;

        if (gps_PositionFix){
            input_AP_Heading = interchip_receive_buffer.pm_data.sp_Heading;
        }
        return TRUE;
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

bool showGains(){
    if (show_gains){
        show_gains = false;
        return true;
    }
    return show_gains;
}

/**
 * @param channel
 * @param gains Order in KP, KI, KD
 */
static void setGains(ControlChannel channel, float* gains){
    setGain(channel, KP, gains[0]);
    setGain(channel, KI, gains[1]);
    setGain(channel, KD, gains[2]);    
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
            case SHOW_GAINS:
                show_gains = true;
                break;
            case SET_PATH_GAIN:
                interchip_send_buffer.am_data.pathGain = CMD_TO_FLOAT(cmd->data);
                interchip_send_buffer.am_data.command = PM_SET_PATH_GAIN;
                sendInterchipData();
                break;
            case SET_ORBIT_GAIN:
                interchip_send_buffer.am_data.orbitGain = CMD_TO_FLOAT(cmd->data);
                interchip_send_buffer.am_data.command = PM_SET_ORBIT_GAIN;
                sendInterchipData();
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
                interchip_send_buffer.am_data.calibrationHeight = CMD_TO_FLOAT(cmd->data);
                interchip_send_buffer.am_data.command = PM_CALIBRATE_ALTIMETER;
                sendInterchipData();
                break;
            case CLEAR_WAYPOINTS:
                interchip_send_buffer.am_data.waypoint.id = *cmd->data; //Dummy Data
                interchip_send_buffer.am_data.command = PM_CLEAR_WAYPOINTS;
                sendInterchipData();
                break;
            case REMOVE_WAYPOINT:
                interchip_send_buffer.am_data.waypoint.id = *cmd->data;
                interchip_send_buffer.am_data.command = PM_REMOVE_WAYPOINT;
                sendInterchipData();
                break;
            case SET_TARGET_WAYPOINT:
                interchip_send_buffer.am_data.waypoint.id = *cmd->data;
                interchip_send_buffer.am_data.command = PM_SET_TARGET_WAYPOINT;
                sendInterchipData();
                break;
            case RETURN_HOME:
                interchip_send_buffer.am_data.command = PM_RETURN_HOME;
                sendInterchipData();
                break;
            case CANCEL_RETURN_HOME:
                interchip_send_buffer.am_data.command = PM_CANCEL_RETURN_HOME;
                sendInterchipData();
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
                interchip_send_buffer.am_data.command = PM_FOLLOW_PATH;
                interchip_send_buffer.am_data.followPath = *cmd->data;
                sendInterchipData();
                break;
            case EXIT_HOLD_ORBIT:
                interchip_send_buffer.am_data.command = PM_EXIT_HOLD_ORBIT;
                sendInterchipData();
                break;
            case SHOW_SCALED_PWM:
                if (*cmd->data == 1){
                    show_scaled_pwm = true;
                } else{
                    show_scaled_pwm = false;
                }
                break;
            case REMOVE_LIMITS:
                limitSetpoint = *(bool*)cmd->data;
            case NEW_WAYPOINT:
                interchip_send_buffer.am_data.waypoint = CMD_TO_TYPE(cmd->data, WaypointWrapper);
                interchip_send_buffer.am_data.command = PM_NEW_WAYPOINT;
                sendInterchipData();
                break;
            case INSERT_WAYPOINT:
                interchip_send_buffer.am_data.waypoint = CMD_TO_TYPE(cmd->data, WaypointWrapper);
                interchip_send_buffer.am_data.command = PM_INSERT_WAYPOINT;
                sendInterchipData();
                break;
            case UPDATE_WAYPOINT:
                interchip_send_buffer.am_data.waypoint = CMD_TO_TYPE(cmd->data, WaypointWrapper);
                interchip_send_buffer.am_data.command = PM_UPDATE_WAYPOINT;
                sendInterchipData();
                break;
            case SET_RETURN_HOME_COORDINATES:
                interchip_send_buffer.am_data.waypoint = CMD_TO_TYPE(cmd->data, WaypointWrapper);
                interchip_send_buffer.am_data.command = PM_SET_RETURN_HOME_COORDINATES;
                sendInterchipData();
                break;
            case TARE_IMU:
                adjustVNOrientationMatrix(CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_IMU:
                setVNOrientationMatrix(CMD_TO_FLOAT_ARRAY(cmd->data));
                break;    
            case SET_ROLL_RATE_GAINS:
                setGains(ROLL_RATE, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_PITCH_RATE_GAINS:
                setGains(PITCH_RATE, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_YAW_RATE_GAINS:
                setGains(YAW_RATE, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_ROLL_ANGLE_GAINS:
                setGains(ROLL_ANGLE, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_PITCH_ANGLE_GAINS:
                setGains(PITCH_ANGLE, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_HEADING_GAINS:
                setGains(HEADING, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_ALTITUDE_GAINS:
                setGains(ALTITUDE, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            case SET_GROUND_SPEED_GAINS:
                setGains(GSPEED, CMD_TO_FLOAT_ARRAY(cmd->data));
                break;
            default:
                break;
        }
       freeDatalinkCommand( cmd );
    }
}

bool writeDatalink(PacketType packet){
    static TelemetryBlock statusData;

    int* input;
    int* output; //Pointers used for RC channel inputs and outputs
    statusData.type = packet;
    
    switch(packet){
        case PACKET_TYPE_POSITION:
            statusData.data.position_block.lat = getLatitude();
            statusData.data.position_block.lon = getLongitude();
            statusData.data.position_block.sys_time = getTime();
            statusData.data.position_block.gps_time = gps_Time; //the utc time
            statusData.data.position_block.pitch = getPitch();
            statusData.data.position_block.roll = getRoll();
            statusData.data.position_block.yaw = getYaw();
            statusData.data.position_block.pitch_rate = getPitchRate();
            statusData.data.position_block.roll_rate = getRollRate();
            statusData.data.position_block.yaw_rate = getYawRate();
            statusData.data.position_block.airspeed = airspeed;
            statusData.data.position_block.altitude = getAltitude();
            statusData.data.position_block.ground_speed = gps_GroundSpeed;
            statusData.data.position_block.heading = getHeading();
            break;
        case PACKET_TYPE_STATUS:
            statusData.data.status_block.roll_rate_setpoint = getRollRateSetpoint();
            statusData.data.status_block.pitch_rate_setpoint = getPitchRateSetpoint();
            statusData.data.status_block.yaw_rate_setpoint = getYawRateSetpoint();
            statusData.data.status_block.roll_setpoint = getRollAngleSetpoint();
            statusData.data.status_block.pitch_setpoint = getPitchAngleSetpoint();
            statusData.data.status_block.heading_setpoint = getHeadingSetpoint();
            statusData.data.status_block.altitude_setpoint = getAltitudeSetpoint();
            statusData.data.status_block.throttle_setpoint = getThrottleSetpoint();
            
            statusData.data.status_block.internal_battery_voltage = batteryLevel1;
            statusData.data.status_block.external_battery_voltage = batteryLevel2;
            
            statusData.data.status_block.autonomous_level = controlLevel;
            statusData.data.status_block.startup_errors = getStartupErrorCodes();
            statusData.data.status_block.am_interchip_errors = getInterchipErrorCount();
            statusData.data.status_block.pm_interchip_errors = pm_interchip_error_count;
            statusData.data.status_block.gps_communication_errors = gps_communication_error_count;
            
            statusData.data.status_block.ul_receive_errors = getRadioReceiveErrors();
            statusData.data.status_block.dl_transmission_errors = getRadioTransmissionErrors();
            statusData.data.status_block.peripheral_status = gps_Satellites + (gps_PositionFix << 4);; //TODO: implement this
            statusData.data.status_block.uhf_channel_status = getPWMInputStatus();
            statusData.data.status_block.ul_rssi = getRadioRSSI();
            statusData.data.status_block.uhf_rssi = getUHFRSSI(getTime());
            statusData.data.status_block.uhf_link_quality = getUHFLinkQuality(getTime());
       
            statusData.data.status_block.program_state = getProgramStatus(); //TODO: modify this
            statusData.data.status_block.waypoint_index = waypointIndex;
            statusData.data.status_block.waypoint_count = waypointCount;
            statusData.data.status_block.path_checksum = waypointChecksum;
            break;
        case PACKET_TYPE_GAINS:
            statusData.data.gain_block.roll_rate_kp = getGain(ROLL_RATE, KP);
            statusData.data.gain_block.roll_rate_kd = getGain(ROLL_RATE, KD);
            statusData.data.gain_block.roll_rate_ki = getGain(ROLL_RATE, KI);
            
            statusData.data.gain_block.pitch_rate_kp = getGain(PITCH_RATE, KP);
            statusData.data.gain_block.pitch_rate_kd = getGain(PITCH_RATE, KD);
            statusData.data.gain_block.pitch_rate_ki = getGain(PITCH_RATE, KI);
            
            statusData.data.gain_block.yaw_rate_kp = getGain(YAW_RATE, KP);
            statusData.data.gain_block.yaw_rate_kd = getGain(YAW_RATE, KD);
            statusData.data.gain_block.yaw_rate_ki = getGain(YAW_RATE, KI);
            
            statusData.data.gain_block.roll_angle_kp = getGain(ROLL_ANGLE, KP);
            statusData.data.gain_block.roll_angle_kd = getGain(ROLL_ANGLE, KD);
            statusData.data.gain_block.roll_angle_ki = getGain(ROLL_ANGLE, KI);
            
            statusData.data.gain_block.pitch_angle_kp = getGain(PITCH_ANGLE, KP);
            statusData.data.gain_block.pitch_angle_kd = getGain(PITCH_ANGLE, KD);
            statusData.data.gain_block.pitch_angle_ki = getGain(PITCH_ANGLE, KI);
            
            statusData.data.gain_block.heading_kp = getGain(HEADING, KP);
            statusData.data.gain_block.heading_ki = getGain(HEADING, KI);
            statusData.data.gain_block.altitude_kp = getGain(ALTITUDE, KP);
            statusData.data.gain_block.altitude_ki = getGain(ALTITUDE, KI);
            statusData.data.gain_block.ground_speed_kp = getGain(GSPEED, KP);
            statusData.data.gain_block.ground_speed_ki = getGain(GSPEED, KI);
            statusData.data.gain_block.path_kp = pmPathGain;
            statusData.data.gain_block.orbit_kp = pmOrbitGain;
            break;
        case PACKET_TYPE_CHANNELS:
            if (show_scaled_pwm){
                input = getPWMArray(getTime());
                output = getPWMOutputs();
                statusData.data.channels_block.channels_scaled = true;
            } else {
                input = (int*)getICValues(getTime());
                output = (int*)getOCValues();
                statusData.data.channels_block.channels_scaled = false;
            }
            statusData.data.channels_block.ch1_in = input[0];
            statusData.data.channels_block.ch2_in = input[1];
            statusData.data.channels_block.ch3_in = input[2];
            statusData.data.channels_block.ch4_in = input[3];
            statusData.data.channels_block.ch5_in = input[4];
            statusData.data.channels_block.ch6_in = input[5];
            statusData.data.channels_block.ch7_in = input[6];
            statusData.data.channels_block.ch8_in = input[7];
            
            statusData.data.channels_block.ch1_out = output[0];
            statusData.data.channels_block.ch2_out = output[1];
            statusData.data.channels_block.ch3_out = output[2];
            statusData.data.channels_block.ch4_out = output[3];
            statusData.data.channels_block.ch5_out = output[4];
            statusData.data.channels_block.ch6_out = output[5];
            statusData.data.channels_block.ch7_out = output[6];
            statusData.data.channels_block.ch8_out = output[7];
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
        interchip_send_buffer.am_data.command = PM_RETURN_HOME;
        sendInterchipData();
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
