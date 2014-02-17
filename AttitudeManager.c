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
#include "main.h"

#if !(PATH_MANAGER && ATTITUDE_MANAGER && COMMUN0ICATION_MANAGER)
#include "InterchipDMA.h"
#endif


#if !(PATH_MANAGER && ATTITUDE_MANAGER && COMMUNICATION_MANAGER)
extern PMData pmData;
#endif

long long time = 0;
long long lastTime = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
    //Temporary Timer Interrupt
    time += 20;
    IFS0bits.T2IF = 0;
    // Clear Timer1 Interrupt Flag
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

//GPS Data
float gps_Heading = 0;
float gps_GroundSpeed = 0; //NOTE: NEEDS TO BE IN METERS/SECOND. CALCULATIONS DEPEND ON THESE UNITS. GPS RETURNS KM/H.
float gps_Time = 0;

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
int control_Roll;
int control_Pitch;
int control_Throttle;
int control_Yaw;

//System constants
float maxRollAngle = 60; // max allowed roll angle in degrees
float maxPitchAngle = 60;
//float maxYawAngle = 20;
float maxHeadingRate = 36; //Max heading change of 36 degrees per second

void attitudeInit() {
    //Debug Mode initialize communication with the serial port (Computer)
    if (DEBUG) {
        InitUART1();
    }

    //Initialize Interchip communication
    init_DMA0();
    init_DMA1();
    init_SPI1();

    /* Initialize IMU with correct orientation matrix and filter settings */
    //IMU position matrix
    float x_angle_offset = (-90 + 40 - 33 + 8) * PI / 180.0; //Degrees
    float y_angle_offset = -0 * PI / 180.0;
    float z_angle_offset = (-10 + 20 - 13) * PI / 180.0;
    float refRotationMatrix[9] = {cos(y_angle_offset) * cos(z_angle_offset), -cos(y_angle_offset) * sin(z_angle_offset), sin(y_angle_offset),
        sin(x_angle_offset) * sin(y_angle_offset) * cos(z_angle_offset) + sin(z_angle_offset) * cos(x_angle_offset), -sin(x_angle_offset) * sin(y_angle_offset) * sin(z_angle_offset) + cos(z_angle_offset) * cos(x_angle_offset), -sin(x_angle_offset) * cos(y_angle_offset),
        -cos(x_angle_offset) * sin(y_angle_offset) * cos(z_angle_offset) + sin(z_angle_offset) * sin(x_angle_offset), cos(x_angle_offset) * sin(y_angle_offset) * sin(z_angle_offset) + cos(z_angle_offset) * sin(x_angle_offset), cos(x_angle_offset) * cos(y_angle_offset)};
    
    float filterVariance[10] = {+1.0e-6, +4.968087236542740e-006, +4.112245302664222e-006, +1.775172462044985e-004, +2.396688069825628e+006, +3.198907908235179e+006, +1.389186225936592e+005, +6.620304667228608e-005, +4.869544197003338e-005, +1.560574584667775e-004};
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
//                   sprintf(str,"%f",pmData.time);
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


    if (DEBUG) {
//                    char str[20];
//                    sprintf(str,"%f",imuData[0]);
//                    UART1_SendString(str);
    }
    /*****************************************************************************
     *****************************************************************************

                                 CONTROL CODE

     *****************************************************************************
     *****************************************************************************/

    if (HEADING_CONTROL){
        //Estimation of Roll angle based on heading:
        //TODO: Add if statement to use rudder for small heading changes
        // -(maxHeadingRate)/180.0,
        sp_HeadingRate = controlSignalHeading(sp_Heading, gps_Heading, gps_Time);
        //Kinematics formula for approximating Roll angle from Heading and Velocity
        sp_RollAngle = atan(gps_GroundSpeed * sp_HeadingRate/9.8);

    }

    if (ORIENTATION_CONTROL) {
        // If we are using accelerometer based stabalization
        // convert sp_xxxRate to an angle (based on maxAngle)
        // If we are getting input from the controller
        // convert sp_xxxxRate to an sp_xxxxAngle in degrees

        if (!HEADING_CONTROL){
//            //sp_YawAngle = sp_YawRate / (sp_Range / maxYawAngle);
            sp_RollAngle = (-sp_RollRate / (SP_RANGE / maxRollAngle) - 1/1.5) * 45.0/50.0;
            sp_PitchAngle = -sp_PitchRate / (SP_RANGE / maxPitchAngle);
        }

        // Output to servos based on requested angle and actual angle (using a gain value)
        // Set this to sp_xxxxRate so that the gyros can do their thing aswell

        //sp_YawRate = controlSignalAngles(sp_YawAngle, imu_YawAngle, kd_Accel_Yaw, -(SP_RANGE) / (maxYawAngle)) ;
        sp_ComputedYawRate = sp_YawRate;
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
            if (sp_GearSwitch > 600) {
                if (sp_Type < 640) {
                    float roll_gain = ((float) (sp_Value - 520)) / (SP_RANGE) * 6 / 1.6212766647 + 2;  //4+ 0.4
                    setGain(ROLL, GAIN_KP, roll_gain < 2.5 ? 1 : roll_gain); //0.4
                    currentGain = (GAIN_KP << 4) + ROLL;
                } else{ //if (sp_Type > 640 && sp_Type < 710) {
                    float pitch_gain = ((float) (sp_Value - 520)) / (SP_RANGE) * 10 / 1.6212766647 + 3; //5 + 0.4
                    setGain(PITCH, GAIN_KP, pitch_gain < 3.5 ? 1 : pitch_gain);  //0.5
                    currentGain = (GAIN_KP << 4) + PITCH;
                    }
//                } else if (sp_Type > 710) {
//                    setGain(YAW, GAIN_KP, ((float) (sp_Value - 520)) / (SP_RANGE) * 4 / 1.6212766647 * 1); //10/0.1
//                    currentGain = (GAIN_KP << 4) + YAW;
//                }

            }
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
        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) * 2 > sp_RollRate - sp_ComputedRollRate) {
            setIntegralSum(ROLL, getIntegralSum(ROLL) / 1.1);
        }
    }
    if (control_Roll < LOWER_PWM) {
        control_Roll = LOWER_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(ROLL) * getGain(ROLL, GAIN_KI) * lastTime * 2 < sp_RollRate - sp_ComputedRollRate) {
            setIntegralSum(ROLL, getIntegralSum(ROLL) / 1.1);
        }
    }
    if (control_Pitch > UPPER_PWM) {
        control_Pitch = UPPER_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(PITCH) * getGain(PITCH, GAIN_KI)* lastTime * 2 > sp_PitchRate - sp_ComputedPitchRate) {
            setIntegralSum(PITCH, getIntegralSum(PITCH) / 1.1);
        }
    }
    if (control_Pitch < LOWER_PWM) {
        control_Pitch = LOWER_PWM;
        // Limits the effects of the integrator, if the output signal is maxed out
        if (getIntegralSum(PITCH) * getGain(PITCH, GAIN_KI) * lastTime * 2 < sp_PitchRate - sp_ComputedPitchRate) {
            setIntegralSum(PITCH, getIntegralSum(PITCH) / 1.1);
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

    if (time - lastTime > 200) {
        lastTime = time;
        struct telem_block* statusData = createTelemetryBlock();
        statusData->millis = time;
        statusData->lat = 0;
        statusData->lon = 0;
        statusData->pitch = imu_PitchAngle;
        statusData->roll = imu_RollAngle;
        statusData->yaw = imu_YawAngle;
        statusData->pitchRate = imu_PitchRate;
        statusData->rollRate = imu_RollRate;
        statusData->yawRate = imu_YawRate;
        statusData->pitch_gain = getGain(PITCH, (currentGain & 0xF0) >> 4);
        statusData->roll_gain = getGain(ROLL, (currentGain & 0xF0) >> 4);
        statusData->yaw_gain = getGain(YAW, (currentGain & 0xF0) >> 4);
        statusData->pitchSetpoint = sp_PitchRate;
        statusData->rollSetpoint = sp_RollRate;
        statusData->yawSetpoint = sp_YawRate;
        statusData->throttleSetpoint = (int) ((float) (control_Throttle - 454) / (890 - 454)*100);
        statusData->editing_gain = ((sp_GearSwitch > 600) * 0xFF) & (currentGain); //(sp_Switch < 600 &&

        if (BLOCKING_MODE) {
            sendTelemetryBlock(statusData);
            destroyTelemetryBlock(statusData);
        } else {

            pushOutboundTelemetryQueue(statusData);
            //statusData->throttleSetpoint = getOutboundQueueLength();
        }

    }
}
