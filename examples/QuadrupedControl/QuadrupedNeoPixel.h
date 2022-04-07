/*
 * QuadrupedNeoPixel.h
 *
 *  Created on: 18.09.2019
 *      Author: Armin
 */

#ifndef _QUADRUPED_NEOPIXEL_H
#define _QUADRUPED_NEOPIXEL_H

#if defined(QUADRUPED_HAS_NEOPIXEL)
#include <NeoPatterns.h>

#define PIN_NEOPIXEL    4
// How many NeoPixels are mounted?
#define NUM_PIXELS      24

#define PIXELS_ON_ONE_BAR      8
#define PIXEL_OFFSET_RIGHT_BAR 0
#define PIXEL_OFFSET_FRONT_BAR PIXELS_ON_ONE_BAR
#define PIXEL_OFFSET_LEFT_BAR (2*PIXELS_ON_ONE_BAR)

void initNeoPatterns();

#if defined(HAS_ADDITIONAL_REMOTE_COMMANDS)
void doPattern1();
void doPattern2();
void doPatternStripes();
void doPatternHeartbeat();
void doPatternFire();
void doRandomMelody();
#endif

void doWipeOutPatterns();

void wipeOutPatternsBlocking();

bool isAtLeastOnePatternActive();

void showPatternSynchronizedWithServos();

void handleAutomaticMovementPattern();
void handleQuadrupedNeoPixelUpdate();
void handleServoTimerInterrupt();

uint16_t getDelayFromSpeed();

extern NeoPatterns QuadrupedNeoPixelBar; // The main 24 pixel bar containing all the other 3 logical NeoPatterns objects.
extern NeoPatterns RightNeoPixelBar;     // 8 Pixel bar at the right
extern NeoPatterns FrontNeoPixelBar;
extern NeoPatterns LeftNeoPixelBar;
extern color32_t sBarBackgroundColorArrayForDistance[]; // // The color background for front distance bar

extern bool sStartOrChangeNeoPatterns;

#endif // #if defined(QUADRUPED_HAS_NEOPIXEL)

#endif // _QUADRUPED_NEOPIXEL_H
