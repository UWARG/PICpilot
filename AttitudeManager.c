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
#include "OrientationControl.h"
#include "AttitudeManager.h"
#include "commands.h"
#include "cameraManager.h"

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
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void){
    //Timer Interrupt used for the control loops and data link
    time += 20;
    IFS0bits.T2IF = 0;
}

// Setpoints (From radio transmitter or autopilot)
int sp_PitchRate = MIDDLE_PWM;
int sp_ThrottleRate = 0;
int sp_YawRate = MIDDLE_PWM;
int sp_RollRate = MIDDLE_PWM;

int sp_ComputedPitchRate = 0;
//int sp_ComputedThrottleRate = 0;
int sp_ComputedRollRate = 0;
int sp_ComputedYawRate = 0;

int sp_Value = 0; //0=Roll, 1= Pitch, 2=Yaw
int sp_Type = 0; //0 = Saved Value, 1 = Edit Mode
int sp_Switch = 0;
int sp_GearSwitch = 0;
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
int pitchTrim = -43;
int yawTrim = 0;

// Control Signals (Output compare value)
int control_Roll = MIDDLE_PWM;
int control_Pitch = MIDDLE_PWM;
int control_Throttle = 0;
int control_Yaw = MIDDLE_PWM;

float scaleFactor = 1.0119; //Change this

char displayGain = 0;
int controlLevel = 0;

int headingCounter = 0;
char altitudeTrigger = 0;

void attitudeInit() {
    //Debug Mode initialize communication with the serial port (Computer)
    if (DEBUG) {
        InitUART1();
    }

    //Initialize Interchip communication
    init_SPI1();
    init_DMA0();
    init_DMA1();


    /* Initialize IMU with correct orientation matrix and filter settings */
    //IMU position matrix
    float filterVariance[10] = {1e-6, 1e-006, 1e-006, 1e-6, 1e2, 1e2, 1e2, 4, 4, 4};
    VN100_initSPI();
    float offset[3] = {-79,0,13};
    setVNOrientationMatrix((float*)&offset);
     VN100_SPI_SetFiltMeasVar(0, (float*)&filterVariance);
    /* Initialize Input Capture and Output Compare Modules */
    if (DEBUG) {
        initIC(0b10001111);
        initOC(0b111111); //Initialize only Output Compare 1,2,3 and 4,5,6
        UART1_SendString("START OF CODE BEFORE WHILE");
    } else {
        initIC(0b11110001);
        initOC(0b111111); //Initialize only Output Compare 1,2,3 and 4,5,6
    }
}

void attitudeManagerRuntime() {

    //Transfer data from PATHMANAGER CHIP
#if !PATH_MANAGER
    if (newDataAvailable){
        newDataAvailable = 0;
        gps_Time = pmData.time;
        gps_Heading = pmData.heading;
        gps_GroundSpeed = pmData.speed * 1000.0/3600.0; //Convert from km/h to m/s
        gps_Altitude = pmData.altitude;
        gps_Longitude = pmData.longitude;
        gps_Latitude = pmData.latitude;
        gps_Satellites = pmData.satellites;
        gps_PositionFix = pmData.positionFix;
        if (controlLevel & ALTITUDE_CONTROL_SOURCE)
            sp_Altitude = pmData.sp_Altitude;
        if (controlLevel & HEADING_CONTROL_SOURCE)
            sp_Heading = pmData.sp_Heading;
    }
#endif


    /*****************************************************************************
     *****************************************************************************

                                INPUT CAPTURE

     *****************************************************************************
     *****************************************************************************/

        int* icTimeDiff;
        icTimeDiff = getICValues();

        if ((ROLL_CONTROL_SOURCE & controlLevel) == 0){
            sp_RollRate = (icTimeDiff[0] - MIDDLE_PWM);
        }
        if ((PITCH_CONTROL_SOURCE & controlLevel) == 0){
            sp_PitchRate = (icTimeDiff[1] - MIDDLE_PWM);
        }
        if ((THROTTLE_CONTROL_SOURCE & controlLevel) == 0)
            sp_ThrottleRate = (icTimeDiff[2]);
        sp_YawRate = (icTimeDiff[3] - MIDDLE_PWM);

//        sp_GearSwitch = icTimeDiff[4];
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
    imu_RollRate = (imuData[IMU_ROLL_RATE]);
    imu_PitchRate = imuData[IMU_PITCH_RATE];
    imu_YawRate = imuData[IMU_YAW_RATE];

    VN100_SPI_GetYPR(0, &imuData[YAW], &imuData[PITCH], &imuData[ROLL]);
    imu_YawAngle = imuData[YAW];
    imu_PitchAngle = imuData[PITCH];
    imu_RollAngle = (imuData[ROLL]);



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
        control_Throttle = sp_ThrottleRate - controlSignalThrottle(sp_PitchAngle, (int)imu_PitchAngle);
        //TODO: Fix decleration of these constants later - Maybe have a  controller.config file specific to each controller or something (make a script to generate this)
        if (control_Throttle > 890){
            control_Throttle = 890;
        }
        else if (sp_ThrottleRate < 454){
            control_Throttle = 454;
        }
    }
    else
        control_Throttle = sp_ThrottleRate;


    if (controlLevel & HEADING_CONTROL_ON){
        //Estimation of Roll angle based on heading:
        //TODO: Add if statement to use rudder for small heading changes

        if (sp_Heading > 360)
            sp_Heading -=360;
        if (sp_Heading < 0)
            sp_Heading +=360;
        // -(maxHeadingRate)/180.0,
            sp_HeadingRate = controlSignalHeading(sp_Heading, gps_Heading);
            //Approximating Roll angle from Heading
            sp_RollAngle = (int)(atan((float)sp_HeadingRate/MAX_ROLL_ANGLE) * 180/PI);

        if (sp_RollAngle > MAX_ROLL_ANGLE)
            sp_RollAngle = MAX_ROLL_ANGLE;
        if (sp_RollAngle < -MAX_ROLL_ANGLE)
            sp_RollAngle = -MAX_ROLL_ANGLE;
    }
    

    // If we are getting input from the controller convert sp_xxxxRate to an sp_xxxxAngle in degrees

    if ((controlLevel & ROLL_CONTROL_SOURCE) == 0 && (controlLevel & HEADING_CONTROL_ON) == 0)
        sp_RollAngle = (int)((-sp_RollRate / ((float)SP_RANGE / MAX_ROLL_ANGLE) - 1/1.5) * 45.0/50.0);
    if ((controlLevel & PITCH_CONTROL_SOURCE) == 0 && (controlLevel & ALTITUDE_CONTROL_ON) == 0)
        sp_PitchAngle = (int)(-sp_PitchRate / ((float)SP_RANGE / MAX_PITCH_ANGLE));

    //sp_YawRate = controlSignalAngles(sp_YawAngle, imu_YawAngle, kd_Accel_Yaw, -(SP_RANGE) / (maxYawAngle)) ;
    //sp_ComputedYawRate = sp_YawRate;

    // Output to servos based on requested angle and actual angle (using a gain value)
    if (controlLevel & ROLL_CONTROL_TYPE || controlLevel & HEADING_CONTROL_ON){
        sp_ComputedRollRate = controlSignalAngles(sp_RollAngle,  (int)imu_RollAngle, ROLL, -(SP_RANGE) / (MAX_ROLL_ANGLE));
    }
    else{
        sp_ComputedRollRate = sp_RollRate;
    }
    if (controlLevel & PITCH_CONTROL_TYPE || controlLevel & ALTITUDE_CONTROL_ON){
        sp_ComputedPitchRate = controlSignalAngles(sp_PitchAngle, (int)imu_PitchAngle, PITCH, -(SP_RANGE) / (MAX_PITCH_ANGLE));
    }
    else{
        sp_ComputedPitchRate = sp_PitchRate;
    }
    sp_ComputedYawRate = sp_YawRate;

    // CONTROLLER INPUT INTERPRETATION CODE
    if (sp_Switch < 600) {
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
    control_Throttle = control_Throttle;
    control_Roll = controlSignal((int)(sp_ComputedRollRate / SERVO_SCALE_FACTOR), (int)imu_RollRate, ROLL);
    control_Pitch = controlSignal((int)(sp_ComputedPitchRate / SERVO_SCALE_FACTOR), (int)imu_PitchRate, PITCH);
    control_Yaw = controlSignal((int)(sp_ComputedYawRate / SERVO_SCALE_FACTOR), (int)imu_YawRate, YAW);

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

    unsigned int pwmTemp = cameraPollingRuntime(gps_Latitude, gps_Longitude, sp_Switch);
    unsigned int gimblePWM = cameraGimbleStabilization(imu_RollAngle);
    // Sends the output signal to the servo motors
    setPWM(1, control_Roll + rollTrim);
    setPWM(2, control_Pitch + pitchTrim);
    setPWM(3, control_Throttle);
    setPWM(4, control_Yaw + yawTrim);
    setPWM(5, pwmTemp);
    setPWM(6, gimblePWM);
//    setPWM(7, (sp_HeadingRate)/10 * SP_RANGE + MIDDLE_PWM - 20);

#if COMMUNICATION_MANAGER
    readDatalink();
    writeDatalink(DATALINK_SEND_FREQUENCY); //pwmTemp>600?0?:0xFFFFFFFF;
#endif
}

#if COMMUNICATION_MANAGER
void readDatalink(void){
    struct command* cmd = popCommand();
    //TODO: Add rudimentary input validation
    if ( cmd ) {
        switch (cmd->cmd) {
            case DEBUG_TEST:             // Debugging command, writes to debug UART
                UART1_SendString( (char*) cmd->data);
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
                break;
            case SET_ORBIT_GAIN:
                amData.orbitGain = *(float*)(&cmd->data);
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
                sp_ThrottleRate = (int)(*(int*)(&cmd->data) / 100.0  * (890 - 454) + 454);
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
            case NEW_WAYPOINT:
                amData.waypoint =  *(WaypointWrapper*)(&cmd->data);
                amData.waypoint.command = PM_NEW_WAYPOINT;
                int i = 0;
                char checksum = 0;
                for (i = 0; i < sizeof(AMData) - 2; i++){
                    checksum += ((char *)&amData)[i];
                }
                amData.checksum = checksum;
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
        struct telem_block* statusData = createTelemetryBlock();//getDebugTelemetryBlock();//createTelemetryBlock();
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
        statusData->throttleSetpoint = (int) ((float) (sp_ThrottleRate - 454) / (890 - 454)*100);
        statusData->altitudeSetpoint = sp_Altitude;
        statusData->altitude = gps_Altitude;
        statusData->cPitchSetpoint = sp_PitchRate;
        statusData->cRollSetpoint = sp_RollRate;
        statusData->cYawSetpoint = sp_YawRate;
        statusData->cThrottleSetpoint = (int) ((float) (sp_ThrottleRate - 454) / (890 - 454)*100);
        statusData->editing_gain = displayGain + ((sp_Switch < 600) << 4);
        statusData->gpsStatus = gps_Satellites + (gps_PositionFix << 4);

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
    float refRotationMatrix[9] = {cos(angleOffset[1]) * cos(angleOffset[2]), -cos(angleOffset[1]) * sin(angleOffset[2]), sin(angleOffset[1]),
        sin(angleOffset[0]) * sin(angleOffset[1]) * cos(angleOffset[2]) + sin(angleOffset[2]) * cos(angleOffset[0]), -sin(angleOffset[0]) * sin(angleOffset[1]) * sin(angleOffset[2]) + cos(angleOffset[2]) * cos(angleOffset[0]), -sin(angleOffset[0]) * cos(angleOffset[1]),
        -cos(angleOffset[0]) * sin(angleOffset[1]) * cos(angleOffset[2]) + sin(angleOffset[2]) * sin(angleOffset[0]), cos(angleOffset[0]) * sin(angleOffset[1]) * sin(angleOffset[2]) + cos(angleOffset[2]) * sin(angleOffset[0]), cos(angleOffset[0]) * cos(angleOffset[1])};

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