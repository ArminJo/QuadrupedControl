/*
 * QuadrupedNeoPixel.cpp
 *
 * This file mainly contains the control of the attached 3 NeoPixel 8 pixel bars.
 * These 3 bars are chained, in order to use only one pin, and are electrically one 24 pixel bar.
 *
 * The NeopPixel updates are synchronized with the ServoEasing updates by overwriting the ServoEasing
 * function handleServoTimerInterrupt()->handleQuadrupedNeoPixelUpdate() in order not to interfere with the servo pulse generation.
 *
 * New automatic movement patterns are triggered by the flag sStartOrChangeNeoPatterns which is evaluated in handleQuadrupedNeoPixelUpdate() in ISR context.
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
 *
 *  QuadrupedControl is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

#include "QuadrupedConfiguration.h"
#if defined(QUADRUPED_HAS_NEOPIXEL)

#include "QuadrupedControlCommands.h"

#include <NeoPatterns.hpp>

#if defined(QUADRUPED_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h"
#include "TinyIRReceiver.h" // for isTinyReceiverIdle()
#endif

#include "QuadrupedServoControl.h"
#include "QuadrupedBasicMovements.h"  // for sMovingDirection

//#define INFO // activate this to see serial info output

void QuadrupedOnPatternCompleteHandler(NeoPatterns *aLedsPtr);
bool sCleanPatternAfterEnd;
bool sCallShowSynchronized; // Flag set e.g. by main loop to show the pattern synchronized (with servos).

NeoPatterns QuadrupedNeoPixelBar = NeoPatterns(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800, &QuadrupedOnPatternCompleteHandler,
        true);
// false -> do not allow show on partial NeoPixel bar
NeoPatterns RightNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_RIGHT_BAR, PIXELS_ON_ONE_BAR, false,
        &QuadrupedOnPatternCompleteHandler, true);
NeoPatterns FrontNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_FRONT_BAR, PIXELS_ON_ONE_BAR, false,
        &QuadrupedOnPatternCompleteHandler, true);
NeoPatterns LeftNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_LEFT_BAR, PIXELS_ON_ONE_BAR, false,
        &QuadrupedOnPatternCompleteHandler, true);

// The color background for front distance bar
color32_t sBarBackgroundColorArrayForDistance[PIXELS_ON_ONE_BAR] = { COLOR32_RED_QUARTER, COLOR32_RED_QUARTER, COLOR32_RED_QUARTER,
COLOR32_YELLOW, COLOR32_YELLOW, COLOR32_GREEN_QUARTER, COLOR32_GREEN_QUARTER, COLOR32_GREEN_QUARTER };

uint16_t getDelayFromSpeed() {
    uint16_t tDelay = 12000 / sQuadrupedServoSpeed;
#ifdef DEBUG
    Serial.print(F("Speed="));
    Serial.print(sQuadrupedServoSpeed);
    Serial.print(F(" Delay="));
    Serial.println(tDelay);
#endif
    return tDelay;
}

void printActivePattern() {
#ifdef INFO
    Serial.print(F("Start pattern "));
    LeftNeoPixelBar.printPatternName(LeftNeoPixelBar.ActivePattern, &Serial);
    Serial.println();
#endif
}

#ifdef HAS_ADDITIONAL_REMOTE_COMMANDS
void doPattern1() {
    RightNeoPixelBar.RainbowCycle(getDelayFromSpeed() / 8);
    LeftNeoPixelBar.RainbowCycle(getDelayFromSpeed() / 8);
    sCleanPatternAfterEnd = true;

    printActivePattern();
}

void doPattern2() {
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.Fade(COLOR32_GREEN_QUARTER, COLOR32_RED_QUARTER, 32, tDelay);
    LeftNeoPixelBar.Fade(COLOR32_RED_QUARTER, COLOR32_GREEN_QUARTER, 32, tDelay);

    printActivePattern();
}

void doPattern3() {
    RightNeoPixelBar.Stripes(COLOR32_GREEN_QUARTER, 2, COLOR32_RED_QUARTER, 2, 128, getDelayFromSpeed());
    LeftNeoPixelBar.Stripes(COLOR32_GREEN_QUARTER, 2, COLOR32_RED_QUARTER, 2, 128, getDelayFromSpeed());

    printActivePattern();
}

void doPatternFire() {
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.Fire(tDelay, 40);
    LeftNeoPixelBar.Fire(tDelay, 40, DIRECTION_DOWN);
    sCleanPatternAfterEnd = true;

    printActivePattern();
}

void doPatternHeartbeat() {
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.Heartbeat(COLOR32_GREEN_HALF, tDelay, 2);
    FrontNeoPixelBar.Heartbeat(COLOR32_BLUE_HALF, tDelay, 2);
    LeftNeoPixelBar.Heartbeat(COLOR32_RED_HALF, tDelay, 2);

    printActivePattern();
}
#endif // HAS_ADDITIONAL_REMOTE_COMMANDS

void wipeOutPatterns() {
    RightNeoPixelBar.ColorWipe(COLOR32_BLACK, getDelayFromSpeed(), FLAG_DO_NOT_CLEAR, DIRECTION_DOWN);
    FrontNeoPixelBar.clear();
    LeftNeoPixelBar.ColorWipe(COLOR32_BLACK, getDelayFromSpeed(), FLAG_DO_NOT_CLEAR);

    printActivePattern();
}

void initNeoPatterns() {
    QuadrupedNeoPixelBar.begin(); // This sets the output pin.
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.ColorWipe(COLOR32_GREEN_QUARTER, tDelay);
    FrontNeoPixelBar.ScannerExtended(COLOR32_BLUE_HALF, 2, tDelay, 2,
    FLAG_SCANNER_EXT_ROCKET | FLAG_SCANNER_EXT_START_AT_BOTH_ENDS);
    LeftNeoPixelBar.ColorWipe(COLOR32_RED_QUARTER, tDelay, 0, DIRECTION_DOWN);
}

void wipeOutPatternsBlocking() {
    wipeOutPatterns();
    while (isAtLeastOnePatternActive()) {
//        sCallShowSynchronized();
#if defined(QUADRUPED_HAS_IR_CONTROL)
        IRDispatcher.delayAndCheckForStop(10);
#else
        delay(10);
#endif
    }
}

void showPatternSynchronized() {
    sCallShowSynchronized = true; // To trigger show() in handleQuadrupedNeoPixelUpdate()
}

bool isAtLeastOnePatternActive() {
    return (RightNeoPixelBar.ActivePattern != PATTERN_NONE || FrontNeoPixelBar.ActivePattern != PATTERN_NONE
            || LeftNeoPixelBar.ActivePattern != PATTERN_NONE);
}

/*
 * The modified overwritten ServoEasing ISR handling function extended for NeoPixel handling.
 * NeoPixels are handled here, since their show() function blocks interrupts
 * and must therefore be synchronized with the servo pulse generation.
 * The interrupt is not disabled after all servos are stopped like in the original function.
 * This enables to call the NeoPixel update function continuously here.
 * Calling of updateAllServos() is controlled by the misused ICNC1 / Input Capture Noise Canceler flag, which is set by ServoEasing :-).
 *
 * Update all servos from list and check if all servos have stopped.
 */
void handleServoTimerInterrupt() {
#if defined(USE_PCA9685_SERVO_EXPANDER)
    // Otherwise it will hang forever in I2C transfer
    interrupts(); // Enable interrupts
#endif
// Check the (misused) ICNC1 flag, which signals that ServoEasing interrupts were enabled again.
    if (TCCR1B & _BV(ICNC1)) {
        // Flag was set -> call update
        if (updateAllServos()) {
            // All servos have stopped here
            // Do not disable interrupt (we need it for NeoPixels), only reset the flag
            TCCR1B &= ~_BV(ICNC1);    // Reset flag
#ifdef INFO
            Serial.println(F("All servos stopped, using interrupts now only for NeoPixels update"));
#endif
        }
    }
    handleQuadrupedNeoPixelUpdate();
}

/*
 * @brief This function checks all patterns for update and calls show() of the underlying 24 pixel bar if required.
 * It is called in ISR context by handleServoTimerInterrupt() since the show() function blocks interrupts
 * and must therefore be synchronized with the servo pulse generation.
 * @return - true if at least one pattern is active.
 */
void handleQuadrupedNeoPixelUpdate() {

#if defined(QUADRUPED_HAS_IR_CONTROL)
    if (isTinyReceiverIdle()) {
#endif
        /*
         * Check for patterns start or update.
         */
        if (sLastActionTypeForNeopatternsDisplay != sCurrentlyRunningAction) {
            sLastActionTypeForNeopatternsDisplay = sCurrentlyRunningAction;
            handleAutomaticMovementPattern(); // To trigger NeoPatterns generation
        }

        if (QuadrupedNeoPixelBar.updateAllPartialPatterns() || sCallShowSynchronized) {
            sCallShowSynchronized = false;
            QuadrupedNeoPixelBar.show();
        }
#if defined(QUADRUPED_HAS_IR_CONTROL)
    }
#endif
}

/*
 * Must be called if one pattern has ended or movement has changed
 */
void handleAutomaticMovementPattern() {
#ifdef INFO
    Serial.print(F("NP current action="));
    Serial.print(sCurrentlyRunningAction);
    Serial.print(" -> ");
#endif
    if (sCurrentlyRunningAction == ACTION_TYPE_STOP) {
#ifdef INFO
        Serial.println(F("Stop, no new pattern"));
#endif
    } else {
        /*
         * Action ongoing. Start or restart pattern according to sCurrentlyRunningAction and other parameter.
         */
        switch (sCurrentlyRunningAction) {
        case ACTION_TYPE_CREEP:
#ifdef INFO
            Serial.println(F("Starting ColorWipe"));
#endif
            RightNeoPixelBar.ColorWipe(Adafruit_NeoPixel::Color(0, NeoPixel::gamma32(sBodyHeight), 0), getDelayFromSpeed(), 0,
                    sMovingDirection);
            LeftNeoPixelBar.ColorWipe(Adafruit_NeoPixel::Color(NeoPixel::gamma32(sBodyHeight), 0, 0), getDelayFromSpeed(), 0,
                    (sMovingDirection + MOVE_DIRECTION_BACKWARD) & MOVE_DIRECTION_MASK);
            break;
        case ACTION_TYPE_TURN:
#ifdef INFO
            Serial.println(F("Starting Stripes"));
#endif
            QuadrupedNeoPixelBar.Stripes(COLOR32_RED_HALF, 2, COLOR32_GREEN_HALF, 2, 100, getDelayFromSpeed(), sMovingDirection);
            break;
        case ACTION_TYPE_TWIST:
#ifdef INFO
            Serial.println(F("Starting Stripes"));
#endif
            QuadrupedNeoPixelBar.Stripes(COLOR32_RED_HALF, 2, COLOR32_GREEN_HALF, 2, 100, getDelayFromSpeed(), sMovingDirection);
            break;
        case ACTION_TYPE_TROT:
#ifdef INFO
            Serial.println(F("Starting Rockets"));
#endif
            RightNeoPixelBar.ScannerExtended(Adafruit_NeoPixel::Color(0, NeoPixel::gamma32(sBodyHeight), 0), 3, getDelayFromSpeed(),
                    0, FLAG_SCANNER_EXT_ROCKET, sMovingDirection);
            LeftNeoPixelBar.ScannerExtended(Adafruit_NeoPixel::Color(NeoPixel::gamma32(sBodyHeight), 0, 0), 3, getDelayFromSpeed(),
                    0, FLAG_SCANNER_EXT_ROCKET, (sMovingDirection + MOVE_DIRECTION_BACKWARD) & MOVE_DIRECTION_MASK);
            break;
        case ACTION_TYPE_ATTENTION:
#ifdef INFO
            Serial.println(F("Starting Heartbeat"));
#endif
            QuadrupedNeoPixelBar.Heartbeat(COLOR32_BLUE_QUARTER, getDelayFromSpeed(), 2, FLAG_DO_NOT_CLEAR);
            break;

        default:
#ifdef INFO
            Serial.println(F("Not yet implemented"));
#endif
            break;
        }
    }
}

/*
 * The completion callback for each pattern
 */
void QuadrupedOnPatternCompleteHandler(NeoPatterns *aLedsPtr) {
#ifdef DEBUG
    Serial.print(F("Offset="));
    Serial.print(aLedsPtr->PixelOffset);
    Serial.print(F(" Pattern \""));
    aLedsPtr->printPatternName(aLedsPtr->ActivePattern, &Serial);
    Serial.println(F("\" finished"));
#endif
    // Reset ActivePattern if no new one will be started.
    aLedsPtr->ActivePattern = PATTERN_NONE;

    handleAutomaticMovementPattern();
    /*
     * Check for cleanup finished pattern
     */
    if (sCleanPatternAfterEnd && aLedsPtr->ActivePattern == PATTERN_NONE) {
        sCleanPatternAfterEnd = false;
        Serial.println(F("Do wipe out"));
        wipeOutPatterns();
    }
}
#endif // #if defined(QUADRUPED_HAS_NEOPIXEL)
