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

float sp_PitchAngle = 0;
//float sp_YawAngle = 0;
float sp_RollAngle = 0;

//Heading Variables
float sp_Heading = 0;
float sp_HeadingRate = 0;

//Altitude Variables
float sp_Altitude = 0;
float sp_GroundSpeed = 0;

//GPS Data
float gps_Heading = 0;
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

// Control Signals (Output compare value)
int control_Roll = MIDDLE_PWM;
int control_Pitch = MIDDLE_PWM;
int control_Throttle = 0;
int control_Yaw = MIDDLE_PWM;

char displayGain = 0;
char controlLevel = 0;

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
    float angleOffset[3] = {deg2rad(-75), deg2rad(0), deg2rad(-3)};
    
    float filterVariance[10] = {+1.0e-6, +4.968087236542740e-006, +4.112245302664222e-006, +1.775172462044985e-004, +2.396688069825628e+006, +3.198907908235179e+006, +1.389186225936592e+005, +6.620304667228608e-002, +4.869544197003338e-002, +1.560574584667775e-001};
    VN100_initSPI();
    setVNOrientationMatrix((float*)&angleOffset);
    VN100_SPI_SetFiltMeasVar(0, (float*)&filterVariance);


    /* Initialize Input Capture and Output Compare Modules */
    if (DEBUG) {
        initIC(0b11111111);
        initOC(0b11111); //Initialize only Output Compare 1,2,3 and 4
        UART1_SendString("START OF CODE BEFORE WHILE");
    } else {
        initIC(0b11110001);
        initOC(0b11111); //Initialize only Output Compare 1,2,3 and 4
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
        if (controlLevel >= FULL_AUTONOMY){
            sp_Altitude = pmData.sp_Altitude;
            sp_Heading = pmData.sp_Heading;
        }
    }
#endif


    /*****************************************************************************
     *****************************************************************************

                                INPUT CAPTURE

     *****************************************************************************
     *****************************************************************************/

    int* icTimeDiff;
    icTimeDiff = getICValues();

    sp_RollRate = (icTimeDiff[0] - MIDDLE_PWM +7);
    sp_PitchRate = (icTimeDiff[1] - MIDDLE_PWM);
    sp_ThrottleRate = (icTimeDiff[2] - MIDDLE_PWM);
    sp_YawRate = (icTimeDiff[3] - MIDDLE_PWM);

    sp_GearSwitch = icTimeDiff[4];
    sp_Type = icTimeDiff[5];
    sp_Value = icTimeDiff[6];
    sp_Switch = icTimeDiff[7];

    if (DEBUG) {
//                   char str[20];
//                   sprintf(str,"%f",time);
//                   UART1_SendString(str);
//                   sprintf(str,"%f",gps_Heading);
//                   UART1_SendString(str);
        
    }

    /*****************************************************************************
     *****************************************************************************

                                IMU COMMUNICATION

     *****************************************************************************
     *****************************************************************************/
    VN100_SPI_GetRates(0, (float*) &imuData);
    //Outputs in order: Roll,Pitch,Yaw
    imu_RollRate = (imuData[ROLL_RATE]);
    imu_PitchRate = imuData[PITCH_RATE];
    imu_YawRate = imuData[YAW_RATE];
    //        imu_PitchRate = imuData[YAW_RATE];
    //        imu_YawRate = -imuData[PITCH_RATE];

    VN100_SPI_GetYPR(0, &imuData[YAW], &imuData[PITCH], &imuData[ROLL]);
    imu_YawAngle = imuData[YAW];
    imu_PitchAngle = imuData[PITCH];
    imu_RollAngle = (imuData[ROLL]);

    //        VN100_SPI_GetMag(0,&imuData);


    /*****************************************************************************
     *****************************************************************************

                                 CONTROL CODE

     *****************************************************************************
     *****************************************************************************/

    if (THROTTLE_CONTROL){
        if (controlLevel >= GROUND_STATION_THROTTLE_CONTROL){
            sp_ThrottleRate = controlSignalThrottle(sp_GroundSpeed, gps_GroundSpeed, time);
            //TODO: Fix decleration of these constants later - Maybe have a  controller.config file specific to each controller or something (make a script to generate this)
            if (sp_ThrottleRate > 890){
                sp_ThrottleRate = 890;
            }
            else if (sp_ThrottleRate < 454){
                sp_ThrottleRate = 454;
            }
        }
    }

    if (ALTITUDE_CONTROL){

        if (controlLevel >= CONTROLLER_ALTITUDE_CONTROL && controlLevel <= CONTROLLER_HEADING_CONTROL){
            if ((sp_Switch < 600) && altitudeTrigger == 0){
                sp_Altitude = gps_Altitude;
                if (controlLevel == CONTROLLER_HEADING_CONTROL)
                    sp_Heading = gps_Heading;
                altitudeTrigger = 1;
            }
            else if (sp_Switch > 600)
                altitudeTrigger = 0;
        }

        sp_PitchAngle = controlSignalAltitude(sp_Altitude,gps_Altitude, time/1000.0);
        if (sp_PitchAngle > MAX_PITCH_ANGLE)
            sp_PitchAngle = MAX_PITCH_ANGLE;
        if (sp_PitchAngle < -MAX_PITCH_ANGLE)
            sp_PitchAngle = -MAX_PITCH_ANGLE;
    }


    if (HEADING_CONTROL){
        //Estimation of Roll angle based on heading:
        //TODO: Add if statement to use rudder for small heading changes


        if (sp_Heading > 360)
            sp_Heading -=360;
        if (sp_Heading < 0)
            sp_Heading +=360;

        if (controlLevel == CONTROLLER_HEADING_CONTROL || controlLevel >= GROUND_STATION_HEADING_CONTROL){
        // -(maxHeadingRate)/180.0,
            sp_HeadingRate = controlSignalHeading(sp_Heading, gps_Heading, time/1000.0);
            //Kinematics formula for approximating Roll angle from Heading and Velocity
            sp_RollAngle = atan(sp_HeadingRate) * 180/PI;

        if (sp_RollAngle > MAX_ROLL_ANGLE)
            sp_RollAngle = MAX_ROLL_ANGLE;
        if (sp_RollAngle < -MAX_ROLL_ANGLE)
            sp_RollAngle = -MAX_ROLL_ANGLE;
        }

        if (DEBUG) {
//                    char str[40];
//                    sprintf(str,"Ground Speed:%f",gps_GroundSpeed);
//                    UART1_SendString(str);
//                    sprintf(str,"Roll Angle:%f",sp_RollAngle);
//                    UART1_SendString(str);
        }
    }

    if (ORIENTATION_CONTROL && controlLevel >= CONTROLLER_ANGLE_CONTROL) {
        // If we are using accelerometer based stabalization
        // convert sp_xxxRate to an angle (based on maxAngle)
        // If we are getting input from the controller
        // convert sp_xxxxRate to an sp_xxxxAngle in degrees

        if (controlLevel == CONTROLLER_ANGLE_CONTROL){
//            //sp_YawAngle = sp_YawRate / (sp_Range / maxYawAngle);
            sp_RollAngle = (-sp_RollRate / (SP_RANGE / MAX_ROLL_ANGLE) - 1/1.5) * 45.0/50.0;
            sp_PitchAngle = -sp_PitchRate / (SP_RANGE / MAX_PITCH_ANGLE);
        }

        // Output to servos based on requested angle and actual angle (using a gain value)
        // Set this to sp_xxxxRate so that the gyros can do their thing aswell

        //sp_YawRate = controlSignalAngles(sp_YawAngle, imu_YawAngle, kd_Accel_Yaw, -(SP_RANGE) / (maxYawAngle)) ;
//        sp_ComputedYawRate = sp_YawRate;
        
        sp_ComputedRollRate = controlSignalAngles(sp_RollAngle,  imu_RollAngle, ROLL, -(SP_RANGE) / (MAX_ROLL_ANGLE),time/1000.0);
        sp_ComputedPitchRate = controlSignalAngles(sp_PitchAngle, imu_PitchAngle, PITCH, -(SP_RANGE) / (MAX_PITCH_ANGLE),time/1000.0);
    } else if (controlLevel >= CONTROLLER_RATE_CONTROL) {
        sp_ComputedRollRate = sp_RollRate;
        sp_ComputedPitchRate = sp_PitchRate;
        sp_ComputedYawRate = sp_YawRate;
    }

    if (!STABILIZATION_CONTROL) {
        control_Roll = sp_RollRate + MIDDLE_PWM;
        control_Pitch = sp_PitchRate + MIDDLE_PWM;
        control_Yaw = sp_YawRate + MIDDLE_PWM;
        control_Throttle = sp_ThrottleRate + MIDDLE_PWM;
    } else {

        // CONTROLLER INPUT INTERPRETATION CODE
        if (sp_Switch < 600) {
            unfreezeIntegral();
//                    if (sp_GearSwitch > 600) {
//                        float roll_gain = ((float) (sp_Value - 520)) / (SP_RANGE) * 10 / 1.6212766647 + 0;  //4+ 0.4
//                        setGain(HEADING, GAIN_KP, roll_gain < 0.5 ? 0 : roll_gain); //0.4
//                        currentGain = (GAIN_KI << 4) + HEADING;
//                        roll_gain = ((float) (sp_Type - 520)) / (SP_RANGE) *  0/ 1.6212766647 + 0; //5 + 0.4
//                        setGain(HEADING, GAIN_KI, roll_gain < 1 ? 0.0 : roll_gain);  //0.5
//                        currentGain = (GAIN_KI << 4) + HEADING;
//                    }
//                    else if (sp_GearSwitch < 600 && sp_GearSwitch > 540){
//                        float pitch_gain = ((float) (sp_Value - 520)) / (SP_RANGE) * 10 / 1.6212766647 + 0.5; //5 + 0.4
//                        setGain(PITCH, GAIN_KP, pitch_gain < 0.5 ? 0.5 : pitch_gain);  //0.5
//                        currentGain = (GAIN_KP << 4) + PITCH;
//                        pitch_gain = ((float) (sp_Type - 520)) / (SP_RANGE) * 0.5 / 1.6212766647 + 0; //5 + 0.4
//                        setGain(PITCH, GAIN_KI, pitch_gain < 0.01 ? 0 : pitch_gain);  //0.5
//                        currentGain = (GAIN_KI << 4) + PITCH;
//                    }
//                    else{
//                        float heading_gain = ((float) (sp_Value - 520)) / (SP_RANGE) * 20 / 1.6212766647 + 10;
//                        setGain(ALTITUDE, GAIN_KD, heading_gain < 10.5 ? 10 : heading_gain);  //0.5
//                        currentGain = (GAIN_KD << 4) + ALTITUDE;
//                        float heading_gain = ((float) (sp_Type - 520)) / (SP_RANGE) * 30/ 1.6212766647 + 0;
//                        setGain(ALTITUDE, GAIN_KP, heading_gain < 1? 0.5 : heading_gain);  //0.5
//                        currentGain = (GAIN_KP << 4) + ALTITUDE;
//                    }

//                } else if (sp_Type > 710) {
//                    setGain(YAW, GAIN_KP, ((float) (sp_Value - 520)) / (SP_RANGE) * 4 / 1.6212766647 * 1); //10/0.1
//                    currentGain = (GAIN_KP << 4) + YAW;
//                }

        } else {
            freezeIntegral();
        }

        // Control Signals (Output compare value)
        control_Throttle = sp_ThrottleRate + MIDDLE_PWM;
        control_Roll = controlSignal(sp_ComputedRollRate / SERVO_SCALE_FACTOR, imu_RollRate, ROLL_RATE);
        control_Pitch = controlSignal(sp_ComputedPitchRate / SERVO_SCALE_FACTOR, imu_PitchRate, PITCH_RATE);
        control_Yaw = controlSignal(sp_ComputedYawRate / SERVO_SCALE_FACTOR, imu_YawRate, YAW_RATE);

    }
    /*****************************************************************************
     *****************************************************************************

                                OUTPUT COMPARE

     *****************************************************************************
     *****************************************************************************/
    if (DEBUG) {

    }
    if (control_Roll > UPPER_PWM) {
        control_Roll = UPPER_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) > sp_RollRate - sp_ComputedRollRate) {
            setIntegralSum(ROLL, getIntegralSum(ROLL)/1.1);
        }
    }
    if (control_Roll < LOWER_PWM) {
        control_Roll = LOWER_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) < sp_RollAngle - sp_ComputedRollRate) {
            setIntegralSum(ROLL, getIntegralSum(ROLL)/1.1);
        }
    }
    if (control_Pitch > UPPER_PWM) {
        control_Pitch = UPPER_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(PITCH) * getGain(PITCH, GAIN_KI) > sp_PitchAngle - sp_ComputedPitchRate) {
            setIntegralSum(PITCH, getIntegralSum(PITCH)/1.1);
        }
    }
    if (control_Pitch < LOWER_PWM) {
        control_Pitch = LOWER_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(PITCH) * getGain(PITCH, GAIN_KI) < sp_PitchAngle - sp_ComputedPitchRate) {
            setIntegralSum(PITCH, getIntegralSum(PITCH)/1.1);
        }
    }

    if (control_Yaw > UPPER_PWM)
        control_Yaw = UPPER_PWM;
    if (control_Yaw < LOWER_PWM)
        control_Yaw = LOWER_PWM;

    float pwmTemp = cameraPollingRuntime(gps_Latitude, gps_Longitude, sp_Switch);
    // Sends the output signal to the servo motors
    setPWM(1, control_Roll);
    setPWM(2, control_Pitch);
    setPWM(3, control_Throttle);
    setPWM(4, control_Yaw);
    setPWM(5, pwmTemp);

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
                break;
            case SET_ORBIT_GAIN:
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
                sp_PitchAngle = *(float*)(&cmd->data);
                break;
            case SET_ROLL_ANGLE:
                sp_RollAngle = *(float*)(&cmd->data);
                break;
            case SET_YAW_ANGLE:
//                sp_YawAngle = *(float*)(&cmd->data);
                break;
            case SET_ALTITUDE:
                sp_Altitude = *(float*)(&cmd->data);
                break;
            case SET_HEADING:
                sp_Heading = *(float*)(&cmd->data);
                break;
            case SET_THROTTLE:
                sp_ThrottleRate = *(int*)(&cmd->data);
                break;
            case TARE_IMU:
                tareVN100();
                break;
            case SET_AUTONOMOUS_LEVEL:
                controlLevel = *(char*)(&cmd->data);
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
        statusData->throttleSetpoint = (int) ((float) (control_Throttle - 454) / (890 - 454)*100);
        statusData->altitudeSetpoint = sp_Altitude;
        statusData->altitude = gps_Altitude;
        statusData->cPitchSetpoint = sp_PitchRate;
        statusData->cRollSetpoint = sp_RollRate;
        statusData->cYawSetpoint = sp_YawRate;
        statusData->cThrottleSetpoint = (int) ((float) (control_Throttle - 454) / (890 - 454)*100);
        statusData->editing_gain = sp_Switch < 600?displayGain + 1:0; //(sp_Switch < 600 &&
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

void tareVN100(void){
    float YPROffset[3];
    VN100_SPI_GetYPR(0, &YPROffset[2], &imuData[1], &imuData[0]);
    setVNOrientationMatrix((float*)&YPROffset);

}

void setVNOrientationMatrix(float* angleOffset){
    //angleOffset[0] = x, angleOffset[1] = y, angleOffset[2] = z
    float refRotationMatrix[9] = {cos(angleOffset[1]) * cos(angleOffset[2]), -cos(angleOffset[1]) * sin(angleOffset[2]), sin(angleOffset[1]),
        sin(angleOffset[0]) * sin(angleOffset[1]) * cos(angleOffset[2]) + sin(angleOffset[2]) * cos(angleOffset[0]), -sin(angleOffset[0]) * sin(angleOffset[1]) * sin(angleOffset[2]) + cos(angleOffset[2]) * cos(angleOffset[0]), -sin(angleOffset[0]) * cos(angleOffset[1]),
        -cos(angleOffset[0]) * sin(angleOffset[1]) * cos(angleOffset[2]) + sin(angleOffset[2]) * sin(angleOffset[0]), cos(angleOffset[0]) * sin(angleOffset[1]) * sin(angleOffset[2]) + cos(angleOffset[2]) * sin(angleOffset[0]), cos(angleOffset[0]) * cos(angleOffset[1])};

    VN100_SPI_SetRefFrameRot(0, (float*)&refRotationMatrix);
    VN100_SPI_WriteSettings(0);
    VN100_SPI_Reset(0);
}
