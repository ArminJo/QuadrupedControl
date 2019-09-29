/*
 * Commands.h
 *
 * list of functions to call by IR command
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef SRC_COMMANDS_H_
#define SRC_COMMANDS_H_

// The code for the called command is available in variable sCurrentIRCode
// All functions have the prefix __attribute__((weak)) in order to enable easy overwriting with own functions.

// Empty functions to be overwritten
void doBeep();
void doTest();

// Basic moves
void doCreepForward();
void doCreepBack();
void doTrot();
void doTurnLeft();
void doTurnRight();

// Combined moves
void doDance();
void doWave();
void doTwist();
void doBow();
void doLeanLeft();
void doLeanRight();
void doLeanBack();
void doLeanFront();

void doAutoMove();
void doAttention();

// Special commands
void doCenterServos();
void doCalibration();

/*
 * Instant command functions
 */
void doStop();
void doSetDirectionForward();
void doSetDirectionBack();
void doSetDirectionRight();
void doSetDirectionLeft();
void doIncreaseSpeed();
void doDecreaseSpeed();
void doIncreaseHeight();
void doDecreaseHeight();
void convertBodyHeightAngleToHeight();

#endif /* SRC_COMMANDS_H_ */

#pragma once
