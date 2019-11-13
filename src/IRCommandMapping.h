/*
 * IRCommandMapping.h
 *
 * IR remote button codes, strings, and functions to call
 *
 *  Created on: 08.03.2019
 *      Author: Armin
 */

#ifndef IR_COMMAND_MAPING_H_
#define IR_COMMAND_MAPING_H_

#include <Arduino.h>
#include "Commands.h" // includes all the commands used in the mapping arrays below

/*
 * !!! Choose your remote !!!
 */
//#define USE_KEYES_REMOTE_CLONE With number pad and direction control switched, will be taken as default
//#define USE_KEYES_REMOTE // The mePed 2 Standard remote
//#define USE_WM10_REMOTE
//#define USE_BLACK_DVD_REMOTE
#if !defined(USE_KEYES_REMOTE) && !defined(USE_WM10_REMOTE) && !defined(USE_KEYES_REMOTE_CLONE) && !defined(USE_BLACK_DVD_REMOTE)
#define USE_KEYES_REMOTE_CLONE // the one you can buy at aliexpress
#endif

#if (defined(USE_KEYES_REMOTE) && defined(USE_WM10_REMOTE)) || (defined(USE_KEYES_REMOTE) && defined(USE_KEYES_REMOTE_CLONE))
#error "Please choose only one remote for compile"
#endif

#define IR_NEC_REPEAT_ADDRESS 0xFFFF
#define IR_NEC_REPEAT_CODE 0x0

#ifdef USE_KEYES_REMOTE_CLONE
#define IR_REMOTE_NAME "KEYES_CLONE"
// Codes for the KEYES CLONE remote control with 17 Keys with keypad above direction control
#define IR_ADDRESS 0xFF00

#define IR_UP    0x18
#define IR_DOWN  0x52
#define IR_RIGHT 0x5A
#define IR_LEFT  0x08
#define IR_OK    0x1C

#define IR_1    0x46
#define IR_2    0x45
#define IR_3    0x47
#define IR_4    0x44
#define IR_5    0x40
#define IR_6    0x43
#define IR_7    0x07
#define IR_8    0x15
#define IR_9    0x09
#define IR_0    0x19

#define IR_STAR 0x16
#define IR_HASH 0x0D
/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_OK
#define COMMAND_STOP        IR_HASH
#define COMMAND_CALIBRATE   IR_0
#define COMMAND_DANCE       IR_1
#define COMMAND_WAVE        IR_3
#define COMMAND_TWIST       IR_7
#define COMMAND_TROT        IR_9
#define COMMAND_AUTO        IR_5
#define COMMAND_TEST        IR_STAR

#define COMMAND_INCREASE_SPEED  IR_6
#define COMMAND_DECREASE_SPEED  IR_4
#define COMMAND_INCREASE_HEIGHT IR_2
#define COMMAND_DECREASE_HEIGHT IR_8

// locally for doCalibration
#define COMMAND_ENTER       IR_OK
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN

/*
 * Special codes not sent by the remote
 */
#define COMMAND_EMPTY       0x99 // code no command received
#define COMMAND_INVALID     0x98 // code for command received, but not in mapping
#endif

#ifdef USE_KEYES_REMOTE
#define IR_REMOTE_NAME "KEYES"
/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the KEYES remote control with 17 Keys
#define IR_ADDRESS 0xFF00

#define IR_UP    0x46
#define IR_DOWN  0x15
#define IR_RIGHT 0x43
#define IR_LEFT  0x44
#define IR_OK    0x40

#define IR_1    0x16
#define IR_2    0x19
#define IR_3    0x0D
#define IR_4    0x0C
#define IR_5    0x18
#define IR_6    0x5E
#define IR_7    0x08
#define IR_8    0x1C
#define IR_9    0x5A
#define IR_0    0x52

#define IR_STAR 0x42
#define IR_HASH 0x4A

/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_OK
#define COMMAND_STOP        IR_HASH
#define COMMAND_CALIBRATE   IR_0
#define COMMAND_DANCE       IR_1
#define COMMAND_WAVE        IR_3
#define COMMAND_TWIST       IR_7
#define COMMAND_TROT        IR_9
#define COMMAND_AUTO        IR_5
#define COMMAND_TEST        IR_STAR

#define COMMAND_INCREASE_SPEED  IR_6
#define COMMAND_DECREASE_SPEED  IR_4
#define COMMAND_INCREASE_HEIGHT IR_2
#define COMMAND_DECREASE_HEIGHT IR_8

// locally for doCalibration
#define COMMAND_ENTER       IR_OK
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN

/*
 * Special codes not sent by the remote
 */
#define COMMAND_EMPTY       0x99 // code no command received
#define COMMAND_INVALID     0x98 // code for command received, but not in mapping
#endif

#ifdef USE_WM10_REMOTE
#define IR_REMOTE_NAME "WM10"
/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the WM010 remote control with 14 Keys
#define IR_ADDRESS 0xF708

#define IR_UP  0x4
#define IR_DOWN 0x51
#define IR_RIGHT 0x8
#define IR_LEFT 0x14
#define IR_ENTER 0x7

#define IR_ON_OFF 0xB
#define IR_MUTE 0x48

#define IR_SRC 0x1
#define IR_RETURN 0x1C

#define IR_VOL_MINUS 0xD
#define IR_VOL_PLUS 0x1D

#define IR_FAST_FORWARD 0x16
#define IR_FAST_BACK 0x59
#define IR_PLAY_PAUSE 0x1F

/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_ENTER
#define COMMAND_STOP        IR_ON_OFF
#define COMMAND_CALIBRATE   IR_MUTE
#define COMMAND_DANCE       IR_SRC
#define COMMAND_WAVE        IR_RETURN
#define COMMAND_TWIST       COMMAND_EMPTY // not on this remote
#define COMMAND_TROT        IR_PLAY_PAUSE
#define COMMAND_AUTO        COMMAND_EMPTY // not on this remote
#define COMMAND_TEST        COMMAND_EMPTY // not on this remote

#define COMMAND_INCREASE_SPEED  IR_VOL_PLUS
#define COMMAND_DECREASE_SPEED  IR_VOL_MINUS
#define COMMAND_INCREASE_HEIGHT IR_FAST_FORWARD
#define COMMAND_DECREASE_HEIGHT IR_FAST_BACK

// locally for doCalibration
#define COMMAND_ENTER       IR_ENTER
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN

/*
 * Special codes not sent by the remote
 */
#define COMMAND_EMPTY       0x99 // code no command received
#define COMMAND_INVALID     0x98 // code for command received, but not in mapping
#endif

#ifdef USE_BLACK_DVD_REMOTE
#define IR_REMOTE_NAME "BLACK_DVD"
#define HAS_ADDITIONAL_REMOTE_COMMANDS

// Codes for black remote control for an old DVD Player
#define IR_ADDRESS 0x7B80

#define IR_ON_OFF 0x13

#define IR_1    0x01
#define IR_2    0x02
#define IR_3    0x03
#define IR_4    0x04
#define IR_5    0x05
#define IR_6    0x06
#define IR_7    0x07
#define IR_8    0x08
#define IR_9    0x09
#define IR_0    0x00

#define IR_CH_PLUS      0x0A
#define IR_CH_MINUS     0x0B

#define IR_REC          0x15
#define IR_PAUSE        0x1A

#define IR_UP           0x16
#define IR_DOWN         0x17
#define IR_RIGHT        0x18
#define IR_LEFT         0x19

#define IR_ENTER        0x45
#define IR_INDEX        0x14
#define IR_CANCEL       0x4A
#define IR_MENU         0x50

// Lower small keys
#define IR_1_LOWER      0x0D // Timer_REC
#define IR_2_LOWER      0x1D
#define IR_3_LOWER      0x5F // Call
#define IR_4_LOWER      0x51
#define IR_5_LOWER      0x4C
#define IR_6_LOWER      0x4B
#define IR_7_LOWER      0x1E
#define IR_8_LOWER      0x12
#define IR_9_LOWER      0x0E // Audio Select
#define IR_EJECT        0x4E
/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_ENTER
#define COMMAND_STOP        IR_ON_OFF
#define COMMAND_CALIBRATE   IR_REC
#define COMMAND_DANCE       IR_1
#define COMMAND_WAVE        IR_3
#define COMMAND_TWIST       IR_7
#define COMMAND_TROT        IR_9
#define COMMAND_AUTO        IR_5
#define COMMAND_TEST        IR_MENU

#define COMMAND_INCREASE_SPEED  IR_6
#define COMMAND_DECREASE_SPEED  IR_4
#define COMMAND_INCREASE_HEIGHT IR_2
#define COMMAND_DECREASE_HEIGHT IR_8

// locally for doCalibration
#define COMMAND_ENTER       IR_ENTER
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN

#define COMMAND_US_RIGHT    IR_CH_PLUS
#define COMMAND_US_SCAN     IR_CH_MINUS
#define COMMAND_US_LEFT     IR_0

#define COMMAND_PATTERN_1   IR_1_LOWER
#define COMMAND_PATTERN_2   IR_2_LOWER
#define COMMAND_PATTERN_3   IR_3_LOWER

#define COMMAND_PATTERN_HEARTBEAT   IR_7_LOWER
#define COMMAND_PATTERN_FIRE        IR_8_LOWER
#define COMMAND_PATTERN_WIPE        IR_EJECT

/*
 * Special codes not sent by the remote
 */
#define COMMAND_EMPTY       0x99 // code no command received
#define COMMAND_INVALID     0x98 // code for command received, but not in mapping
#endif
/*
 * This is valid for all remotes above
 */
#define IR_REPEAT_ADDRESS IR_NEC_REPEAT_ADDRESS
#define IR_REPEAT_CODE IR_NEC_REPEAT_CODE

/*
 * THIRD:
 * Main mapping of commands to C functions
 */

// IR strings of functions for output
static const char beep[] PROGMEM ="beep";
static const char forward[] PROGMEM ="forward";
static const char back[] PROGMEM ="back";
static const char enter[] PROGMEM ="enter";
static const char center[] PROGMEM ="center";
static const char right[] PROGMEM ="right";
static const char left[] PROGMEM ="left";
static const char dirForward[] PROGMEM ="dir forward";
static const char dirBack[] PROGMEM ="dir back";
static const char dirRight[] PROGMEM ="dir right";
static const char dirLeft[] PROGMEM ="dir left";
static const char increaseSpeed[] PROGMEM ="increase speed";
static const char decreaseSpeed[] PROGMEM ="decrease speed";
static const char increaseHeight[] PROGMEM ="increase height";
static const char decreaseHeight[] PROGMEM ="decrease height";
static const char wave[] PROGMEM ="wave";
static const char calibration[] PROGMEM ="calibration";
//static const char onOff[] PROGMEM ="on/off";
static const char stop[] PROGMEM ="stop";
static const char dance[] PROGMEM ="dance";
static const char trot[] PROGMEM ="trot";
static const char twist[] PROGMEM ="twist";
static const char autoMove[] PROGMEM ="auto move";
static const char myMove[] PROGMEM ="my move";
static const char ultrasonicServo[] PROGMEM ="US servo";
static const char test[] PROGMEM ="test";
static const char pattern[] PROGMEM ="pattern";
static const char unknown[] PROGMEM ="unknown";

#define IR_COMMAND_FLAG_NO_REPEAT_EXCLUSIVE 0x00 // default
#define IR_COMMAND_FLAG_ACCEPT_REPEAT       0x01 // repeat accepted
#define IR_COMMAND_FLAG_NOT_EXCLUSIVE       0x02 // Command can be processed any time
#define IR_COMMAND_FLAG_NOT_EXCLUSIVE_REPEATABLE (IR_COMMAND_FLAG_ACCEPT_REPEAT | IR_COMMAND_FLAG_NOT_EXCLUSIVE)

// Basic mapping structure
struct IRToCommandMapping {
    uint8_t IRCode;
    uint8_t Flags;
    void (*CommandToCall)();
    const char * CommandString;
};

/*
 * Main mapping array of commands to C functions and command strings
 * These commands
 */
const struct IRToCommandMapping IRMapping[] = { {
COMMAND_DANCE, IR_COMMAND_FLAG_NO_REPEAT_EXCLUSIVE, &doDance, dance }, {
COMMAND_TWIST, IR_COMMAND_FLAG_NO_REPEAT_EXCLUSIVE, &doTwist, twist }, {
COMMAND_WAVE, IR_COMMAND_FLAG_NO_REPEAT_EXCLUSIVE, &doWave, wave }, {
COMMAND_TROT, IR_COMMAND_FLAG_NO_REPEAT_EXCLUSIVE, &doTrot, trot }, {
COMMAND_AUTO, IR_COMMAND_FLAG_NO_REPEAT_EXCLUSIVE, &doQuadrupedAutoMove, autoMove }, {
COMMAND_TEST, IR_COMMAND_FLAG_NO_REPEAT_EXCLUSIVE, &doTest, test }, {
COMMAND_CENTER, IR_COMMAND_FLAG_NO_REPEAT_EXCLUSIVE, &doCenterServos, center }, {
#if defined(QUADRUPED_HAS_IR_CONTROL) && !defined(USE_USER_DEFINED_MOVEMENTS)
COMMAND_CALIBRATE, IR_COMMAND_FLAG_NO_REPEAT_EXCLUSIVE, &doCalibration, calibration }, {
#endif
/*
 * Non exclusive commands, set directions
 */
COMMAND_FORWARD, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doSetDirectionForward, dirForward }, {
COMMAND_BACKWARD, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doSetDirectionBack, dirBack }, {
COMMAND_RIGHT, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doSetDirectionRight, dirRight }, {
COMMAND_LEFT, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doSetDirectionLeft, dirLeft }, {
/*
 * Repeatable commands
 */
COMMAND_INCREASE_SPEED, IR_COMMAND_FLAG_NOT_EXCLUSIVE_REPEATABLE, &doIncreaseSpeed, increaseSpeed }, {
COMMAND_DECREASE_SPEED, IR_COMMAND_FLAG_NOT_EXCLUSIVE_REPEATABLE, &doDecreaseSpeed, decreaseSpeed }, {
COMMAND_INCREASE_HEIGHT, IR_COMMAND_FLAG_NOT_EXCLUSIVE_REPEATABLE, &doIncreaseHeight, increaseHeight }, {
COMMAND_DECREASE_HEIGHT, IR_COMMAND_FLAG_NOT_EXCLUSIVE_REPEATABLE, &doDecreaseHeight, decreaseHeight }, {
COMMAND_STOP, IR_COMMAND_FLAG_NOT_EXCLUSIVE_REPEATABLE, &doStop, stop }

#ifdef HAS_ADDITIONAL_REMOTE_COMMANDS
/*
 * Commands not accessible by simple remote because of lack of keys
 */
        , { COMMAND_US_RIGHT, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doUSRight, ultrasonicServo }, {
        COMMAND_US_LEFT, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doUSLeft, ultrasonicServo }, {
        COMMAND_US_SCAN, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doUSScan, ultrasonicServo }, {
        COMMAND_PATTERN_1, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doPattern1, pattern }, {
        COMMAND_PATTERN_2, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doPattern2, pattern }, {
        COMMAND_PATTERN_3, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doPattern3, pattern }, {
        COMMAND_PATTERN_HEARTBEAT, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doPatternHeartbeat, pattern }, {
        COMMAND_PATTERN_FIRE, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &doPatternFire, pattern }, {
        COMMAND_PATTERN_WIPE, IR_COMMAND_FLAG_NOT_EXCLUSIVE, &wipeOutPatterns, pattern }

#endif
        };

#endif /* IR_COMMAND_MAPING_H_ */

#pragma once
