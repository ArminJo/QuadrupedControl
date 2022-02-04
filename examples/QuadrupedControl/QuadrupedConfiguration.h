/*
 * QuadrupedConfiguration.h
 *
 *   Contains the feature definitions as well as the pin layout
 *
 *  Created on: 15.09.2019
 *      Author: Armin
 */

#ifndef QUADRUPED_CONFIGURATION_H_
#define QUADRUPED_CONFIGURATION_H_

#define PIN_BUZZER     3

#define VCC_STOP_THRESHOLD_MILLIVOLT 3500 // stop moving if below 3.5 volt

//#define QUADRUPED_HAS_IR_CONTROL      // 8600 bytes (including the movements)
//#define QUADRUPED_PLAYS_RTTTL         // 1350 bytes for Short + Down
//#define QUADRUPED_HAS_NEOPIXEL        // 9400 bytes
//#define QUADRUPED_HAS_US_DISTANCE     // 830 bytes
//#define QUADRUPED_HAS_US_DISTANCE_SERVO
//#define INFO                          // 2850 bytes

/*
 * !!! Choose your remote !!!
 */
//#define USE_KEYES_REMOTE_CLONE With number pad and direction control switched, will be taken as default
//#define USE_KEYES_REMOTE // The mePed 2 Standard remote
//#define USE_LAFVIN_REMOTE // Another name (printed on the remote) for the mePed 2 Standard remote
//#define USE_WM10_REMOTE
//#define USE_WHITE_DVD_REMOTE
/*
 * Choose some predefined configurations
 */
//#define QUADRUPED_1_WITH_DVD_REMOTE
//#define QUADRUPED_2_WITH_LAVWIN_REMOTE
//#define QUADRUPED_3_WITH_KEYES_CLONE_REMOTE
#if defined(QUADRUPED_1_WITH_DVD_REMOTE)
#define USE_WHITE_DVD_REMOTE
#define QUADRUPED_HAS_IR_CONTROL
#define QUADRUPED_PLAYS_RTTTL
#define QUADRUPED_HAS_NEOPIXEL
#define QUADRUPED_HAS_US_DISTANCE
#define QUADRUPED_HAS_US_DISTANCE_SERVO
#define INFO

// This patterns only accessible by DVD remote
#define HAS_ADDITIONAL_REMOTE_COMMANDS
#define ENABLE_PATTERN_RAINBOW_CYCLE
#define ENABLE_PATTERN_FADE
#define ENABLE_PATTERN_FIRE
#endif

#if defined(QUADRUPED_2_WITH_LAVWIN_REMOTE)
#define USE_KEYES_REMOTE // The mePed 2 Standard remote
#define QUADRUPED_HAS_IR_CONTROL
#define QUADRUPED_PLAYS_RTTTL
#define QUADRUPED_HAS_NEOPIXEL
#define QUADRUPED_HAS_US_DISTANCE
#define INFO
#endif

#if defined(QUADRUPED_3_WITH_KEYES_CLONE_REMOTE)
#define USE_KEYES_REMOTE_CLONE
#define QUADRUPED_HAS_IR_CONTROL
#define QUADRUPED_PLAYS_RTTTL
#define QUADRUPED_HAS_NEOPIXEL
#define INFO
#endif

//#define USE_USER_DEFINED_MOVEMENTS

#if defined(QUADRUPED_HAS_IR_CONTROL)
#define USE_TINY_IR_RECEIVER // must be specified before including IRCommandDispatcher.hpp to define which IR library to use
#define IR_INPUT_PIN  A0
#endif

#if defined(QUADRUPED_HAS_US_DISTANCE)
#include "HCSR04.h"

#define PIN_TRIGGER_OUT     A3
#define PIN_ECHO_IN         A4
#define PIN_US_SERVO        13

#define MILLIS_BETWEEN_MEASUREMENTS 200 // 5 per second
#  if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
#include "QuadrupedServoControl.h"
extern Servo USServo;
#  endif
#endif

#if defined(QUADRUPED_PLAYS_RTTTL)
#include <PlayRtttl.h>
#endif

#if defined(QUADRUPED_HAS_NEOPIXEL)
// patterns always used if Neopixel are enabled
#define ENABLE_PATTERN_HEARTBEAT
#define ENABLE_PATTERN_COLOR_WIPE
#define ENABLE_PATTERN_STRIPES
#define ENABLE_PATTERN_SCANNER_EXTENDED
#include "QuadrupedNeoPixel.h"
#endif

/*
 * Is used by the library
 */
bool delayAndCheckForLowVoltageAndStop(uint16_t aDelayMillis);

#endif /* QUADRUPED_CONFIGURATION_H_ */
#pragma once
