/*
 * QuadrupedNeoPixel.cpp
 *
 * This file mainly contains the control of the attached 3 NeoPixel 8 pixel bars.
 * These 3 bars are chained, in order to use only one pin, and are electrically one 24 pixel bar.
 *
 * The NeopPixel updates are synchronized with the ServoEasing updates
 * by overwriting the ServoEasing function handleServoTimerInterrupt()/handleQuadrupedNeoPixelUpdate() in order not to interfere with the servo pulse generation.
 *
 * New automatic movement patterns are triggered by the flag sStartOrChangeNeoPatterns which is evaluated in handleQuadrupedNeoPixelUpdate() in ISR context.
 *
 *  Copyright (C) 2019  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ServoEasing https://github.com/ArminJo/QuadrupedControl.
 *
 *  ServoEasing is free software: you can redistribute it and/or modify
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

#include "QuadrupedNeoPixel.h"
#include "QuadrupedControl.h"
#include "Commands.h"

#if defined(QUADRUPED_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h"
#endif

#include "QuadrupedServoControl.h"
#include "QuadrupedMovements.h"  // for sMovingDirection

//#define INFO // comment this out to see serial info output

void QuadrupedOnPatternCompleteHandler(NeoPatterns * aLedsPtr);
bool sCleanPatternAfterEnd;
bool sStartOrChangeNeoPatterns; // Flag set e.g. by main loop after calling a main command. Flag is read by ISR, to check for new patterns.
bool sCallShowSynchronized; // Flag set e.g. by main loop to show the pattern synchronized (with servos).

NeoPatterns QuadrupedNeoPixelBar = NeoPatterns(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800, &QuadrupedOnPatternCompleteHandler,
        true);
// false -> do not allow show on partial NeoPixel bar
NeoPatterns RightNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_RIGHT_BAR, PIXELS_ON_ONE_BAR,
        &QuadrupedOnPatternCompleteHandler, false, true);
NeoPatterns FrontNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_FRONT_BAR, PIXELS_ON_ONE_BAR,
        &QuadrupedOnPatternCompleteHandler, false, true);
NeoPatterns LeftNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_LEFT_BAR, PIXELS_ON_ONE_BAR,
        &QuadrupedOnPatternCompleteHandler, false, true);

uint16_t getDelayFromSpeed() {
    uint16_t tDelay = 12000 / sServoSpeed;
#ifdef INFO
    Serial.print(F("Speed="));
    Serial.print(sServoSpeed);
    Serial.print(F(" Delay="));
    Serial.println(tDelay);
#endif
    return tDelay;
}

void doPattern1() {
    QuadrupedNeoPixelBar.RainbowCycle(getDelayFromSpeed() / 8);
    sCleanPatternAfterEnd = true;
}
void doPattern2() {
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.Fade(COLOR32_GREEN_QUARTER, COLOR32_RED_QUARTER, 32, tDelay);
    LeftNeoPixelBar.Fade(COLOR32_RED_QUARTER, COLOR32_GREEN_QUARTER, 32, tDelay);
}

void doPattern3() {
    QuadrupedNeoPixelBar.Stripes(COLOR32_GREEN_QUARTER, 4, COLOR32_RED_QUARTER, 4, 128, getDelayFromSpeed());
}

void doPatternFire() {
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.Fire(tDelay, 40);
    LeftNeoPixelBar.Fire(tDelay, 40);
    sCleanPatternAfterEnd = true;
}

void doPatternHeartbeat() {
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.Heartbeat(COLOR32_GREEN_HALF, tDelay, 3);
    FrontNeoPixelBar.Heartbeat(COLOR32_BLUE_HALF, tDelay, 3);
    LeftNeoPixelBar.Heartbeat(COLOR32_RED_HALF, tDelay, 3);
}

void initNeoPatterns() {
    QuadrupedNeoPixelBar.begin(); // This sets the output pin.
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.ColorWipe(COLOR32_GREEN_QUARTER, tDelay);
    FrontNeoPixelBar.ScannerExtended(COLOR32_BLUE_HALF, 2, tDelay, 2,
    FLAG_SCANNER_EXT_ROCKET | FLAG_SCANNER_EXT_START_AT_BOTH_ENDS);
    LeftNeoPixelBar.ColorWipe(COLOR32_RED_QUARTER, tDelay, 0, DIRECTION_DOWN);
}

void wipeOutPatterns() {
    RightNeoPixelBar.ColorWipe(COLOR32_BLACK, getDelayFromSpeed(), FLAG_DO_NOT_CLEAR, DIRECTION_DOWN);
    FrontNeoPixelBar.clear();
    LeftNeoPixelBar.ColorWipe(COLOR32_BLACK, getDelayFromSpeed(), FLAG_DO_NOT_CLEAR);
}

void wipeOutPatternsBlocking() {
    wipeOutPatterns();
    while (isAtLeastOnePatternActive()) {
//        sCallShowSynchronized();
        delayAndCheck(10);
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
 * The modified servo ISR extended for NeoPixel handling.
 * NeoPixels are handled here, since their show() function blocks interrupts
 * and must therefore be synchronized with the servo pulse generation.
 * The interrupt is not disabled, after all servos are stopped in order to call the NeoPixel update function continuously.
 * Calling of updateAllServos() is controlled by the misused ICNC1 / Input Capture Noise Canceler flag, which is set by ServoEasing.
 *
 * Update all servos from list and check if all servos have stopped.
 * Can not call yield() here, since we are in an ISR context here.
 */
void handleServoTimerInterrupt() {
#if defined(USE_PCA9685_SERVO_EXPANDER)
    // Otherwise it will hang forever in I2C transfer
    sei();
#endif
// Check the (misused) ICNC1 flag, which signals that ServoEasing interrupts were enabled again.
    if (TCCR1B & _BV(ICNC1)) {
        // Flag was set -> call update
        if (updateAllServos()) {
            // All servos have stopped here
            // Do not disable interrupt (we need it for NeoPixels), only reset the flag
            // disableServoEasingInterrupt(); // original ISR
            TCCR1B &= ~_BV(ICNC1);    // Only reset flag
#ifdef INFO
            Serial.println(F("Do not disable interrupt, reset only the interrupt flag"));
#endif
        }
    }
    handleQuadrupedNeoPixelUpdate();
}

/*
 * @brief This function checks all patterns for update and calls show() of the underlying 24 pixel bar if needed.
 * It is called in ISR context by handleServoTimerInterrupt() since the show() function blocks interrupts
 * and must therefore be synchronized with the servo pulse generation.
 * @return - true if at least one pattern is active.
 */
bool handleQuadrupedNeoPixelUpdate() {

    /*
     * Check for patterns start or update.
     */
    if (sLastActionType != sActionType) {
        sLastActionType = sActionType;
        handleAutomaticMovementPattern(); // To trigger NeoPatterns generation
    }

    bool tNeedShow = sCallShowSynchronized;
    if (tNeedShow) {
        sCallShowSynchronized = false;
    }

    bool tAtLeastOnePatternActive = false;
    if (RightNeoPixelBar.ActivePattern != PATTERN_NONE) {
        tAtLeastOnePatternActive = true;
        tNeedShow |= RightNeoPixelBar.update();
    }
    if (FrontNeoPixelBar.ActivePattern != PATTERN_NONE) {
        tAtLeastOnePatternActive = true;
        tNeedShow |= FrontNeoPixelBar.update();
    }
    if (LeftNeoPixelBar.ActivePattern != PATTERN_NONE) {
        tAtLeastOnePatternActive = true;
        tNeedShow |= LeftNeoPixelBar.update();
    }
    if (QuadrupedNeoPixelBar.ActivePattern != PATTERN_NONE) {
        // One pattern for all 3 bars here
        QuadrupedNeoPixelBar.update();
    } else if (tNeedShow) {
        QuadrupedNeoPixelBar.show();
    }
    return tAtLeastOnePatternActive;
}

/*
 * Must be called if one pattern has ended or movement has changed
 */
void handleAutomaticMovementPattern() {
#ifdef INFO
    Serial.print(F("Current action="));
    Serial.print(sActionType);
    Serial.print(" -> ");
#endif
    if (sActionType == ACTION_TYPE_STOP) {
#ifdef INFO
        Serial.println(F("No new pattern"));
#endif
    } else {
        /*
         * Action ongoing. Start or restart pattern according to sActionType and other parameter.
         */
        switch (sActionType) {
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
            QuadrupedNeoPixelBar.Heartbeat(COLOR32_BLUE_QUARTER, getDelayFromSpeed(), 3, FLAG_DO_NOT_CLEAR);
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
void QuadrupedOnPatternCompleteHandler(NeoPatterns * aLedsPtr) {
#ifdef INFO
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
