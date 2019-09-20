/*
 * FullQuadrupedControl.cpp
 *
 * Program for controlling a mePed Robot V2 with 8 servos, 3 NeoPixel bars and a HCSR04 US distance module.
 * The IR Remote is attached at pin A0. Supported IR remote are:
 *          KEYES (the original mePed remote)
 *          KEYES_CLONE (the one with numberpad and direction control swapped, which you get when you buy a KEYES at aliexpress).
 *          WM10
 * If you use another than the KEYES_CLONE, you have to select the one you use at line 20ff. in IRCommandMapping.h
 *
 * To run this example you need to install the "ServoEasing", "IRLremote", "PinChangeInterrupt", "NeoPatterns", "Adafruit_NeoPixel" libraries.
 * These libraries can be installed under "Tools -> Manage Libraries..." or "Ctrl+Shift+I".
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

#include "FullQuadrupedControl.h"
#include "QuadrupedNeoPixel.h"

#include "IRCommandDispatcher.h"
#include "QuadrupedServoControl.h"
#include "QuadrupedMovements.h"
#include "IRCommandMapping.h" // for IR_REMOTE_NAME

#include "HCSR04.h"
#include "ADCUtils.h"

/*
 * The auto move function. Put your own moves here.
 *
 * Servos available:
 *  frontLeftPivotServo, frontLeftLiftServo
 *  backLeftPivotServo, backLeftLiftServo
 *  backRightPivotServo, backRightLiftServo
 *  frontRightPivotServo, frontRightLiftServo
 *
 * Useful commands:
 *
 * sMovingDirection = MOVE_DIRECTION_FORWARD; // MOVE_DIRECTION_LEFT etc.
 * moveCreep(1);
 * centerServos();
 * moveTrot(1);
 * moveTurn(1);
 * doLeanLeft();
 * doLeanRight();
 * basicTwist(30);
 * doWave();
 * delayAndCheckIRInput(1000);
 *
 * To move the front left lift servo use:
 * frontLeftLiftServo.easeTo(LIFT_MIN_ANGLE);
 * setLiftServos(LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE);
 * setPivotServos(100, 100, 80, 80);
 */

/*
 * comment this out and put your own code here
 */
//void doAutoMove() {
//    return;
//}

/*
 * Create your own basic movement here
 */
void doTest() {

}

#define VERSION_EXAMPLE "3.0"
// 3.0 NeoPixel and distance sensor added
// 2.1 auto move
// 2.0 major refactoring
// 1.1 mirror computation at transformAndSetPivotServos and transformOneServoIndex

/*
 * Code starts here
 */
void setup() {
    // initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial)
        ; //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    setupQuadrupedServos();
    setSpeedForAllServos(sServoSpeed);

    // Just for setting channel and reference
    getVCCVoltageMillivoltSimple();

    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);

    /*
     * set servo to 90 degree WITHOUT trim and wait
     */
    resetServosTo90Degree();

#if defined(PIN_SPEAKER)
    tone(PIN_SPEAKER, 2000, 300);
#endif
    delay(2000);

    /*
     * set servo to 90 degree with trim and wait
     */
    eepromReadAndSetServoTrim();
    resetServosTo90Degree();
    delay(2000);

    /*
     * Set to initial height
     */
    centerServos();
    convertBodyHeightAngleToHeight();

    setupIRDispatcher();
    Serial.print(F("Listening to IR remote of type "));
    Serial.println(IR_REMOTE_NAME);

    enableServoEasingInterrupt(); // This enables the interrupt, which synchronizes the NeoPixel update with the servo pulse generation.

    initNeoPatterns();
}

void loop() {
    /*
     * Reset moving characteristic
     */
    setEasingTypeToLinear();

    /*
     * Check for IR commands and execute them.
     * Returns only AFTER finishing of requested movement
     */
    loopIRDispatcher();

    /*
     * Do auto move if timeout after boot was reached and no IR command was received
     */
    if (!sAtLeastOneValidIRCodeReceived && (millis() > MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE)) {
        doAutoMove();
        sAtLeastOneValidIRCodeReceived = true; // do auto move only once
    }

    /*
     * Get attention that no command was received since 2 minutes and quadruped may be switched off
     */
    if (millis() - sLastTimeOfValidIRCodeReceived > MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE) {
        doAttention();
        // next attention in 1 minute
        sLastTimeOfValidIRCodeReceived += MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE;
    }

    if (checkForLowVoltage()) {
        shutdownServos();
#if defined(PIN_SPEAKER)
        tone(PIN_SPEAKER, 2000, 200);
        delay(400);
        tone(PIN_SPEAKER, 1400, 300);
        delay(600);
        tone(PIN_SPEAKER, 1000, 400);
        delay(800);
        tone(PIN_SPEAKER, 700, 500);
#endif
        delay(10000);  // wait for next check
    }

//    handleUSSensor();

}

/*
 * Get front distance
 */
void handleUSSensor() {
    static uint16_t sLastDistance;
    uint16_t tDistance = getUSDistanceAsCentiMeter();
    if (tDistance != sLastDistance) {
        Serial.print(F("Distance="));
        Serial.print(tDistance);
        Serial.println(F("cm"));
    }
}

/*
 * Stop servos if voltage gets low
 * @return  true - if voltage too low
 */
bool checkForLowVoltage() {
    uint16_t tVCC = getVCCVoltageMillivoltSimple();
    if (tVCC > VCC_STOP_THRESHOLD_MILLIVOLT) {
        return false; // signal OK
    }
    /*
     * Low voltage here
     */
    Serial.print(F("VCC "));
    Serial.print(tVCC);
    Serial.print(F(" below 3600 Millivolt -> "));
    return true;
}

void doBeep() {
    tone(PIN_SPEAKER, 2000, 200);
    delayAndCheck(400);
    tone(PIN_SPEAKER, 2000, 200);
}

// this overwrites a dummy function of the library
bool checkOncePerDelay() {
    return checkForLowVoltage();
}
