/*
 * QuadrupedNeoPixel.cpp
 *
 * This file mainly contains the control of the attached 3 NeoPixel 8 pixel bars.
 * These 3 bars are chained, in order to use only one pin, and are electrically one 24 pixel bar.
 *
 * The NeopPixel updates are synchronized with the ServoEasing updates
 * by overwriting the ServoEasing function handleServoTimerInterrupt()/handleQuadrupedNeoPixelUpdate() in order not to interfere with the servo pulse generation.
 *
 * New patterns are triggered by the flag sStartOrChangeNeoPatterns which is evaluated in handleQuadrupedNeoPixelUpdate() in ISR context.
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

#if defined(QUADRUPED_IR_CONTROL)
#include "IRCommandDispatcher.h"
#endif

#include "QuadrupedServoControl.h"
#include "QuadrupedMovements.h"  // for sMovingDirection

void QuadrupedPatterns(NeoPatterns * aLedsPtr);
bool sStartOrChangeNeoPatterns; // Flag set e.g. by main loop after calling a main command. Flag is read by ISR, to check for new patterns.
bool sCallShowSynchronized; // Flag set e.g. by main loop to show the pattern synchronized (with servos).

#if defined(NUM_PIXELS)
NeoPatterns QuadrupedNeoPixelBar = NeoPatterns(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800, &QuadrupedPatterns);
// false -> do not allow show on partial NeoPixel bar
NeoPatterns RightNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_RIGHT_BAR, PIXELS_ON_ONE_BAR, &QuadrupedPatterns,
        false);
NeoPatterns FrontNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_FRONT_BAR, PIXELS_ON_ONE_BAR, &QuadrupedPatterns,
        false);
NeoPatterns LeftNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_LEFT_BAR, PIXELS_ON_ONE_BAR, &QuadrupedPatterns,
        false);
#endif

void initNeoPatterns() {
    QuadrupedNeoPixelBar.begin(); // This sets the output pin.
    RightNeoPixelBar.ColorWipe(COLOR32_GREEN_QUARTER, sServoSpeed);
    FrontNeoPixelBar.ScannerExtended(COLOR32_BLUE_HALF, 2, sServoSpeed, 2,
    FLAG_SCANNER_EXT_ROCKET | FLAG_SCANNER_EXT_START_AT_BOTH_ENDS);
    LeftNeoPixelBar.ColorWipe(COLOR32_RED_QUARTER, sServoSpeed, 0, DIRECTION_DOWN);
}

void clearPatternsSlowly() {
    RightNeoPixelBar.ColorWipe(COLOR32_BLACK, sServoSpeed, FLAG_DO_NOT_CLEAR, DIRECTION_DOWN);
    FrontNeoPixelBar.clear();
    LeftNeoPixelBar.ColorWipe(COLOR32_BLACK, sServoSpeed, FLAG_DO_NOT_CLEAR);
}

void clearPatternsSlowlyBlocking() {
    clearPatternsSlowly();
    while (isAtLeastOnePatternActive()) {
//        sCallShowSynchronized();
        delayAndCheck(10);
    }
}
/*
 * This let the Servo ISR look if a new pattern is applicable
 */
void triggerNeoPatterns() {
    sStartOrChangeNeoPatterns = true; // To trigger NeoPatterns generation
}

void showPatternSynchronized() {
    sCallShowSynchronized = true; // To trigger show() in handleQuadrupedNeoPixelUpdate()
}

bool isAtLeastOnePatternActive() {
    return (RightNeoPixelBar.ActivePattern != PATTERN_NONE || FrontNeoPixelBar.ActivePattern != PATTERN_NONE
            || LeftNeoPixelBar.ActivePattern != PATTERN_NONE);
}
/*
 * @brief This function checks all three patterns for update and calls show of the underlying 24 pixel bar if needed.
 * It is called by the servo interrupt function below since the show() function blocks interrupts
 * and must therefore be synchronized with the servo pulse generation.
 * @return - true if at least one pattern is active.
 */
bool handleQuadrupedNeoPixelUpdate() {

    /*
     * Check for patterns start or update.
     */
    if (sStartOrChangeNeoPatterns) {
        sStartOrChangeNeoPatterns = false;
        handleMovementPattern();
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
            Serial.println(F("Do not disable interrupt, reset only the interrupt flag"));
        }
    }
    handleQuadrupedNeoPixelUpdate();
}

/*
 * Must be called if pattern has ended or must be changed
 */
void handleMovementPattern() {
    Serial.print(F("Action="));
    Serial.println(sActionType);
    if (sActionType != ACTION_TYPE_STOP) {
        /*
         * Action ongoing. Start or restart pattern according to sActionType and other parameter.
         */
        switch (sActionType) {
        case ACTION_TYPE_CREEP:
            Serial.println(F("Starting ColorWipe"));
            RightNeoPixelBar.ColorWipe(Adafruit_NeoPixel::Color(0, NeoPixel::gamma5(sBodyHeight), 0), sServoSpeed * 4, 0,
                    sMovingDirection);
            LeftNeoPixelBar.ColorWipe(Adafruit_NeoPixel::Color(NeoPixel::gamma5(sBodyHeight), 0, 0), sServoSpeed * 4, 0,
                    (sMovingDirection + MOVE_DIRECTION_BACKWARD) & MOVE_DIRECTION_MASK);
            break;
        case ACTION_TYPE_TURN:
            Serial.println(F("Starting Stripes"));
            QuadrupedNeoPixelBar.Stripes(COLOR32_RED_HALF, 2, COLOR32_GREEN_HALF, 2, sServoSpeed, 100, sMovingDirection);
            break;
        case ACTION_TYPE_TWIST:
            Serial.println(F("Starting Stripes"));
            QuadrupedNeoPixelBar.Stripes(COLOR32_RED_HALF, 2, COLOR32_GREEN_HALF, 2, sServoSpeed, 100, sMovingDirection);
            break;
        default:
            Serial.println(F("One show only"));
            break;
        }
    }
}

/*
 * The completion callback for each pattern
 */
void QuadrupedPatterns(NeoPatterns * aLedsPtr) {
    Serial.print(F("Pattern="));
    Serial.print(aLedsPtr->ActivePattern);
    Serial.println(F(" finished"));

    handleMovementPattern();
}
