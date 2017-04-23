/*
 * File:   main.h
 * Author: Chris Hajduk
 *
 * Created on October 15, 2013, 8:14 PM
 *
 * This file contains all GLOBAL constants needed for the code.
 * Avoiding adding stuff here unless you really need to.
 *
 */

//Include Libraries
#include "../Common/Common.h"
#include "delay.h"

/**
 * Whether to enable the logger. Disabling it will cause all the functions defined
 * in the module to do nothing
 */
#define DEBUG 0

//Define this for competition, includes higher restrictions for safety
#define COMP_MODE 0

#define FIXED_WING 0
#define MULTIROTOR 1
#define VTOL 2 // TODO

/* CHANGE THIS HEADER FILE WHEN MODIFYING THIS FOR A NEW PLANE OR VEHICLE */
/* Also update configurations in FixedWing.h or Multirotor.h */
#define VEHICLE_TYPE FIXED_WING

#if VEHICLE_TYPE == FIXED_WING
#include "FixedWing.h"
#elif VEHICLE_TYPE == MULTIROTOR
#include "Multirotor.h"
#endif

#if DEBUG
    #include "../Common/Utilities/Logger.h"
#endif
