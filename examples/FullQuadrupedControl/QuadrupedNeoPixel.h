/*
 * QuadrupedNeoPixel.h
 *
 *  Created on: 18.09.2019
 *      Author: Armin
 */

#ifndef QUADRUPED_NEOPIXEL_H_
#define QUADRUPED_NEOPIXEL_H_

#include "NeoPatterns.h" // Click here to get the library: http://librarymanager/All#NeoPatterns

#define PIN_NEOPIXEL    4
// How many NeoPixels are mounted?
#define NUM_PIXELS      24

#define PIXELS_ON_ONE_BAR      8
#define PIXEL_OFFSET_RIGHT_BAR 0
#define PIXEL_OFFSET_FRONT_BAR PIXELS_ON_ONE_BAR
#define PIXEL_OFFSET_LEFT_BAR (2*PIXELS_ON_ONE_BAR)

void doPattern1();

void initNeoPatterns();
void wipeOutPatterns();
void wipeOutPatternsBlocking();

bool isAtLeastOnePatternActive();

void showPatternSynchronized();

void handleAutomaticMovementPattern();
void handleQuadrupedNeoPixelUpdate();
void handleServoTimerInterrupt();

uint16_t getDelayFromSpeed();

extern NeoPatterns QuadrupedNeoPixelBar; // The main 24 pixel bar containing all the other 3 logical NeoPatterns objects.
extern NeoPatterns RightNeoPixelBar;     // 8 Pixel bar at the right
extern NeoPatterns FrontNeoPixelBar;
extern NeoPatterns LeftNeoPixelBar;

extern bool sStartOrChangeNeoPatterns;

#endif /* QUADRUPED_NEOPIXEL_H_ */

#pragma once
