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
#include "main.h"

#if !(PATH_MANAGER && ATTITUDE_MANAGER && COMMUNICATION_MANAGER)
#include "InterchipDMA.h"
#endif


#if !(PATH_MANAGER && ATTITUDE_MANAGER && COMMUNICATION_MANAGER)
extern PMData pmData;
extern AMData amData;
extern char newDataAvailable;
#endif

long int chipTime, lastChipTime;

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void){
    //Temporary Timer Interrupt
    chipTime += 20;
    IFS0bits.T2IF = 0;
    // Clear Timer1 Interrupt Flag
}

float time = 0;
float lastTime = 0;

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

//System constants
float maxRollAngle = 60; // max allowed roll angle in degrees
float maxPitchAngle = 60;
//float maxYawAngle = 20;
float maxHeadingRate = 36; //Max heading change of 36 degrees per second

int tempCounter = 0;
char trigger = 0;

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
    float x_angle_offset = (-90 + 40 - 33 + 8) * PI / 180.0; //Degrees
    float y_angle_offset = -0 * PI / 180.0;
    float z_angle_offset = (-10 + 20 - 13) * PI / 180.0;
    float refRotationMatrix[9] = {cos(y_angle_offset) * cos(z_angle_offset), -cos(y_angle_offset) * sin(z_angle_offset), sin(y_angle_offset),
        sin(x_angle_offset) * sin(y_angle_offset) * cos(z_angle_offset) + sin(z_angle_offset) * cos(x_angle_offset), -sin(x_angle_offset) * sin(y_angle_offset) * sin(z_angle_offset) + cos(z_angle_offset) * cos(x_angle_offset), -sin(x_angle_offset) * cos(y_angle_offset),
        -cos(x_angle_offset) * sin(y_angle_offset) * cos(z_angle_offset) + sin(z_angle_offset) * sin(x_angle_offset), cos(x_angle_offset) * sin(y_angle_offset) * sin(z_angle_offset) + cos(z_angle_offset) * sin(x_angle_offset), cos(x_angle_offset) * cos(y_angle_offset)};
    
    float filterVariance[10] = {+1.0e-6, +4.968087236542740e-006, +4.112245302664222e-006, +1.775172462044985e-004, +2.396688069825628e+006, +3.198907908235179e+006, +1.389186225936592e+005, +6.620304667228608e-002, +4.869544197003338e-002, +1.560574584667775e-001};
    VN100_initSPI();
    VN100_SPI_SetRefFrameRot(0, (float*)&refRotationMatrix);
    VN100_SPI_SetFiltMeasVar(0, (float*)&filterVariance);


//    angle_zero[PITCH] = 0;
//    angle_zero[ROLL] = 0; //-90
//    angle_zero[YAW] = 0; //50



    /* Initialize Input Capture and Output Compare Modules */
    if (DEBUG) {
        initIC(0b11111111);
        initOC(0b1111); //Initialize only Output Compare 1,2,3 and 4
        UART1_SendString("START OF CODE BEFORE WHILE");
    } else {
        initIC(0b11110001);
        initOC(0b1111); //Initialize only Output Compare 1,2,3 and 4
    }
}

void attitudeManagerRuntime() {

    //Transfer data from SPI
    if (newDataAvailable){
        newDataAvailable = 0;
        time = pmData.time;
        gps_Heading = pmData.heading;
        gps_GroundSpeed = pmData.speed * 1000.0/3600.0; //Convert from km/h to m/s
        gps_Altitude = pmData.altitude;
        gps_Longitude = pmData.longitude;
        gps_Latitude = pmData.latitude;
        gps_Satellites = pmData.satellites;
        gps_PositionFix = pmData.positionFix;
//        sp_Altitude = pmData.sp_Altitude;
    }



    /*****************************************************************************
     *****************************************************************************

                                INPUT CAPTURE

     *****************************************************************************
     *****************************************************************************/

    int* icTimeDiff;
    icTimeDiff = getICValues();

    sp_RollRate = (icTimeDiff[0] - MIDDLE_PWM +7);
    sp_PitchRate = (icTimeDiff[1] - MIDDLE_PWM);
    sp_ThrottleRate = (icTimeDiff[2]);
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

    if (ALTITUDE_CONTROL){

        if ((sp_Switch < 600) && trigger == 0){
            sp_Altitude = gps_Altitude;
            sp_Heading = gps_Heading;
            trigger = 1;
        }
        else if (sp_Switch > 600)
            trigger = 0;

        sp_PitchAngle = controlSignalAltitude(sp_Altitude,gps_Altitude, time);
        if (sp_PitchAngle > 20.0)
            sp_PitchAngle = 20.0;
        if (sp_PitchAngle < -20.0)
            sp_PitchAngle = -20.0;
    }


    if (HEADING_CONTROL){
        //Estimation of Roll angle based on heading:
        //TODO: Add if statement to use rudder for small heading changes
//        if (sp_YawRate > SP_RANGE * 0.5 && tempCounter >= 250){
//            tempCounter = 0;
//            sp_Heading += 90;
//        }
//
//        if (sp_YawRate < SP_RANGE * 0.5 && tempCounter >= 250){
//            tempCounter = 0;
//            sp_Heading -= 90;
//        }
//        else
//            tempCounter++;


        
//        sp_Heading += sp_YawRate * 0.20/(-SP_RANGE) > 0.1||sp_YawRate * 0.20/(-SP_RANGE) < -0.1?sp_YawRate * 0.20/(-SP_RANGE):0;

        if (sp_Heading > 360)
            sp_Heading -=360;
        if (sp_Heading < 0)
            sp_Heading +=360;

        // -(maxHeadingRate)/180.0,
        sp_HeadingRate = controlSignalHeading(sp_Heading, gps_Heading, time) * (maxHeadingRate)/180.0;
        //Kinematics formula for approximating Roll angle from Heading and Velocity
        sp_RollAngle = atan(sp_HeadingRate) * 180/PI *0.3;

        if (DEBUG) {
//                    char str[40];
//                    sprintf(str,"Ground Speed:%f",gps_GroundSpeed);
//                    UART1_SendString(str);
//                    sprintf(str,"Roll Angle:%f",sp_RollAngle);
//                    UART1_SendString(str);
        }
    }

    if (ORIENTATION_CONTROL) {
        // If we are using accelerometer based stabalization
        // convert sp_xxxRate to an angle (based on maxAngle)
        // If we are getting input from the controller
        // convert sp_xxxxRate to an sp_xxxxAngle in degrees

        if (!HEADING_CONTROL){
//            //sp_YawAngle = sp_YawRate / (sp_Range / maxYawAngle);
            sp_RollAngle = (-sp_RollRate / (SP_RANGE / maxRollAngle) - 1/1.5) * 45.0/50.0;
//            sp_PitchAngle = -sp_PitchRate / (SP_RANGE / maxPitchAngle);
        }

        // Output to servos based on requested angle and actual angle (using a gain value)
        // Set this to sp_xxxxRate so that the gyros can do their thing aswell

        //sp_YawRate = controlSignalAngles(sp_YawAngle, imu_YawAngle, kd_Accel_Yaw, -(SP_RANGE) / (maxYawAngle)) ;
//        sp_ComputedYawRate = sp_YawRate;
        
        sp_ComputedRollRate = controlSignalAngles(sp_RollAngle,  imu_RollAngle, ROLL, -(SP_RANGE) / (maxRollAngle),time);
        sp_ComputedPitchRate = controlSignalAngles(sp_PitchAngle, imu_PitchAngle, PITCH, -(SP_RANGE) / (maxPitchAngle),time);
    } else {
        sp_ComputedRollRate = sp_RollRate;
        sp_ComputedPitchRate = sp_PitchRate;
        sp_ComputedYawRate = sp_YawRate;
    }

    if (!STABILIZATION_CONTROL) {
        control_Roll = sp_RollRate + MIDDLE_PWM;
        control_Pitch = sp_PitchRate + MIDDLE_PWM;
        control_Yaw = sp_YawRate + MIDDLE_PWM;
        control_Throttle = sp_ThrottleRate;
    } else {

        // CONTROLLER INPUT INTERPRETATION CODE
        if (sp_Switch < 600) {
            unfreezeIntegral();
//                    if (sp_GearSwitch > 600) {
                        float roll_gain = ((float) (sp_Value - 520)) / (SP_RANGE) * 10 / 1.6212766647 + 0;  //4+ 0.4
                        setGain(HEADING, GAIN_KP, roll_gain < 0.5 ? 0 : roll_gain); //0.4
                        currentGain = (GAIN_KI << 4) + HEADING;
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
                        float heading_gain = ((float) (sp_Type - 520)) / (SP_RANGE) * 30/ 1.6212766647 + 0;
                        setGain(ALTITUDE, GAIN_KP, heading_gain < 1? 0.5 : heading_gain);  //0.5
                        currentGain = (GAIN_KP << 4) + ALTITUDE;
//                    }

//                } else if (sp_Type > 710) {
//                    setGain(YAW, GAIN_KP, ((float) (sp_Value - 520)) / (SP_RANGE) * 4 / 1.6212766647 * 1); //10/0.1
//                    currentGain = (GAIN_KP << 4) + YAW;
//                }

        } else {
            freezeIntegral();
        }

        // Control Signals (Output compare value)
        control_Throttle = sp_ThrottleRate;
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


    // Sends the output signal to the servo motors
    setPWM(1, control_Roll);
    setPWM(2, control_Pitch);
    setPWM(3, control_Throttle);
    setPWM(4, control_Yaw);

//    if (time - lastTime < 0){
//        lastTime -= 240000;
//    }

    struct command* cmd = popCommand();
    float floatReceived;
    if ( cmd ) {
        floatReceived = 1.21;
        switch (cmd->cmd) {
            case 0:             // Debugging command, writes to debug UART
                UART1_SendString( (char*) cmd->data);
                break;
            case SET_ROLLGAIN:
                floatReceived = *(float*)(&cmd->data);
                break;
            default:
                break;
        }
        destroyCommand( cmd );
    }

    if (chipTime - lastChipTime > 500) {
        lastChipTime = chipTime;
        struct telem_block* statusData = getDebugTelemetryBlock();//createTelemetryBlock();
        statusData->lat = gps_Latitude;
        statusData->lon = gps_Longitude;
        statusData->millis = time;
        statusData->pitch = imu_PitchAngle;
        statusData->roll = imu_RollAngle;
        statusData->yaw = imu_YawAngle;
        statusData->pitchRate = imu_PitchRate;
        statusData->rollRate = imu_RollRate;
        statusData->yawRate = imu_YawRate;
        statusData->pitch_gain = sp_GearSwitch <= 600?getGain(ALTITUDE, GAIN_KD):getGain(ROLL_RATE, GAIN_KD);
        statusData->roll_gain = getGain(ALTITUDE, GAIN_KP);//sp_GearSwitch <= 600?getGain(ALTITUDE, GAIN_KP):getGain(ROLL, GAIN_KP);
        statusData->yaw_gain = getGain(HEADING, GAIN_KP);//sp_GearSwitch <= 600?getGain(ALTITUDE, GAIN_KI):getGain(ROLL, GAIN_KI);
        statusData->heading = gps_Heading;
        statusData->groundSpeed = gps_GroundSpeed;
        statusData->pitchSetpoint = sp_PitchAngle;
        statusData->rollSetpoint = sp_RollAngle;
        statusData->headingSetpoint = sp_Heading;
        statusData->throttleSetpoint = (int) ((float) (control_Throttle - 454) / (890 - 454)*100);
        statusData->altitudeSetpoint = sp_Altitude;
        statusData->altitude = gps_Altitude;
        //statusData->cPitchSetpoint = sp_PitchRate;
        statusData->cRollSetpoint = sp_RollRate;
        statusData->cYawSetpoint = sp_YawRate;
        statusData->cThrottleSetpoint = (int) ((float) (control_Throttle - 454) / (890 - 454)*100);
        statusData->editing_gain = sp_Switch < 600?(((sp_GearSwitch > 600) * 0xFF) & (currentGain)) + 1:0; //(sp_Switch < 600 &&
        statusData->gpsStatus = gps_Satellites + (gps_PositionFix << 4);

        if (BLOCKING_MODE) {
            sendTelemetryBlock(statusData);
            destroyTelemetryBlock(statusData);
        } else {

            pushOutboundTelemetryQueue(statusData);
            //statusData->throttleSetpoint = getOutboundQueueLength();
        }

    }
}
//TODO: Make a set of functions to override the path manager. (Example: Altitude needs to manually be changed)
