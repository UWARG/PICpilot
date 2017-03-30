/**
 * @file OrientationControl.h
 * @author Ian Frosst
 * @date March 2, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "main.h"
#include "PID.h"

#ifndef ORIENTATIONCONTROL_H
#define	ORIENTATIONCONTROL_H

#define CONTROL_CHANNELS 8

#define MAX_ROLL_ANGLE 35.f // degrees
#define MAX_PITCH_ANGLE 35.f

#define MAX_ROLL_RATE 180.f // degrees/second
#define MAX_PITCH_RATE 180.f
#define MAX_YAW_RATE 240.f

// used for wrapping heading control. converts an angle to -180/180
#define wrap_180(x) (x < -180 ? x + 360 : (x > 180 ? x - 360 : x))

typedef enum {
    KP = 0,
    KI,
    KD
} GainType;

typedef enum {
    ROLL_RATE = 0,
    PITCH_RATE,
    YAW_RATE,
    ROLL_ANGLE,
    PITCH_ANGLE,
    HEADING,
    ALTITUDE,
    GSPEED
} ControlChannel;

/**
 * Initializes all the basic orientation PID controllers
 */
void orientationInit();

/**
 * Retrieves a pointer to one of the basic orientation PID controllers
 * @param channel Channel to retrieve
 * @return Pointer to PIDVal struct for that channel
 */
PIDVal* getPID(ControlChannel channel);

/**
 * Gets a gain value from a PID channel
 * @param channel Channel to get gain from
 * @param type Type of gain (KP, KI or KD)
 * @return Requested gain value
 */
float getGain(ControlChannel channel, GainType type);

/**
 * Sets a gain value for a PID channel
 * @param channel Channel to set gain for
 * @param type Type of gain (KP, KI or KD)
 * @param value Value to set the gain to
 */
void setGain(ControlChannel channel, GainType type, float value);

bool areGainsUpdated();
void forceGainUpdate();

#endif	/* ORIENTATIONCONTROL_H */
