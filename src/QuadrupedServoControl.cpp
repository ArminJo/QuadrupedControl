/*
 * QuadrupedServoControl.cpp
 *
 * Contains all the servo related functions and data.
 *
 * Program for controlling a mePed Robot V2 with 8 servos using an IR Remote at pin A0
 * Supported IR remote are KEYES (the original mePed remote) and WM10
 * Select the one you have at line 23 in QuadrupedIRConfiguration.h
 *
 * To run this example need to install the "ServoEasing", "IRLremote" and "PinChangeInterrupt" libraries under "Tools -> Manage Libraries..." or "Ctrl+Shift+I"
 * Use "ServoEasing", "IRLremote" and "PinChangeInterrupt" as filter string.
 *
 *  Copyright (C) 2019  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
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

#include "QuadrupedServoConfiguration.h"
#include "QuadrupedServoControl.h"
#include "IRCommandDispatcher.h" // for delayAndCheck()


//#include "IRCommandDispatcher.h" // for checkIRInput(); and RETURN_IF_STOP;

// Define 8 servos in exact this order!
ServoEasing frontLeftPivotServo;    // 0 - Front Left Pivot Servo
ServoEasing frontLeftLiftServo;     // 1 - Front Left Lift Servo
ServoEasing backLeftPivotServo;     // 2 - Back Left Pivot Servo
ServoEasing backLeftLiftServo;      // 3 - Back Left Lift Servo
ServoEasing backRightPivotServo;    // 4 - Back Right Pivot Servo
ServoEasing backRightLiftServo;     // 5 - Back Right Lift Servo
ServoEasing frontRightPivotServo;   // 6 - Front Right Pivot Servo
ServoEasing frontRightLiftServo;    // 7 - Front Right Lift Servo

uint16_t sServoSpeed = 90;      // in degree/second
uint8_t sBodyHeightAngle = LIFT_MIN_ANGLE + 20; // From LIFT_MIN_ANGLE to LIFT_MAX_ANGLE !!! The bigger the angle, the lower the body !!!
uint8_t sBodyHeight;  // normalized body height from 0 (low) to 255 (high)

// Arrays of trim angles stored in EEPROM
EEMEM int8_t sServoTrimAnglesEEPROM[NUMBER_OF_SERVOS]; // The one which resides in EEPROM and IR read out at startup - filled by eepromWriteServoTrim
int8_t sServoTrimAngles[NUMBER_OF_SERVOS]; // RAM copy for easy setting trim angles by remote, filled by eepromReadServoTrim

void setupQuadrupedServos() {
    // Attach servos to Arduino Pins
    frontLeftPivotServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN);
    frontLeftLiftServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 1);
    backLeftPivotServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 2);
    backLeftLiftServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 3);
    // Invert direction for lift servos.
    backLeftLiftServo.setReverseOperation(true);
    backRightPivotServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 4);
    backRightLiftServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 5);
    frontRightPivotServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 6);
    frontRightLiftServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 7);
    frontRightLiftServo.setReverseOperation(true);
}

void shutdownServos() {
    Serial.println(F("Shutdown servos"));
    sBodyHeightAngle = LIFT_MAX_ANGLE;
    centerServos();
}

void centerServos() {
    setAllServos(90, 90, 90, 90, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle);
}

void setSpeed(uint16_t aSpeed) {
    sServoSpeed = aSpeed;
    setSpeedForAllServos(sServoSpeed);
    printSpeed();
}

void printSpeed() {
    Serial.print(F(" Speed="));
    Serial.println(sServoSpeed);
}

void printTrimAngles() {
    for (uint8_t i = 0; i < NUMBER_OF_SERVOS; ++i) {
        Serial.print(F("ServoTrimAngle["));
        Serial.print(i);
        Serial.print(F("]="));
        Serial.println(sServoTrimAngles[i]);
        sServoArray[i]->setTrim(sServoTrimAngles[i]);
    }
}

void resetServosTo90Degree() {
    for (uint8_t i = 0; i < NUMBER_OF_SERVOS; ++i) {
        sServoArray[i]->write(90);
    }
}

/*
 * Copy calibration array from EEPROM to RAM and set uninitialized values to 0
 */
void eepromReadAndSetServoTrim() {
    Serial.println(F("eepromReadAndSetServoTrim()"));
    eeprom_read_block((void*) &sServoTrimAngles, &sServoTrimAnglesEEPROM, NUMBER_OF_SERVOS);
    printTrimAngles();
}

void eepromWriteServoTrim() {
    eeprom_write_block((void*) &sServoTrimAngles, &sServoTrimAnglesEEPROM, NUMBER_OF_SERVOS);
    printTrimAngles();
}

void setEasingTypeToLinear() {
    for (uint8_t tServoIndex = 0; tServoIndex < NUMBER_OF_SERVOS; ++tServoIndex) {
        sServoArray[tServoIndex]->setEasingType(EASE_LINEAR);
    }
}

void setEasingTypeForMoving() {
    for (int tServoIndex = 0; tServoIndex < NUMBER_OF_SERVOS; ++tServoIndex) {
        sServoArray[tServoIndex]->setEasingType(EASE_LINEAR);
        tServoIndex++;
        sServoArray[tServoIndex]->setEasingType(EASE_QUADRATIC_BOUNCING);
    }
}

/*
 * Main transformation routines
 *
 * Direction forward changes nothing.
 * Direction backward swaps forward and backward servos / increases index by NUMBER_OF_LEGS/2
 * Direction left increases index by 1 and right by 3.
 * Mirroring swaps left and right (XOR with 0x06) and invert all angles.
 */

uint8_t getMirrorXorMask(uint8_t aDirection) {
// XOR the index with this value to get the mirrored index
    if (aDirection & MOVE_DIRECTION_SIDE_MASK) {
        return 0x2;
    } else {
        return 0x6;
    }
}

void transformAndSetAllServos(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot, int aFrontRightPivot,
        int aFrontLeftLift, int aBackLeftLift, int aBackRightLift, int aFrontRightLift, uint8_t aDirection, bool doMirror,
        bool aDoMove) {
    uint8_t tIndexToAdd = aDirection * SERVOS_PER_LEG;
    uint8_t tXorToGetMirroredIndex = 0x0;
// Invert angles for pivot servos
    bool doInvert = false;
    if (doMirror) {
// XOR the index with this value to get the mirrored index
        tXorToGetMirroredIndex = getMirrorXorMask(aDirection);
        doInvert = true;
    }

    uint8_t tEffectivePivotServoIndex;
    tEffectivePivotServoIndex = ((FRONT_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFrontLeftPivot = 180 - aFrontLeftPivot;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aFrontLeftPivot;
    sServoNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aFrontLeftLift;

    tEffectivePivotServoIndex = ((BACK_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBackLeftPivot = 180 - aBackLeftPivot;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aBackLeftPivot;
    sServoNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aBackLeftLift;

    tEffectivePivotServoIndex = ((BACK_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBackRightPivot = 180 - aBackRightPivot;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aBackRightPivot;
    sServoNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aBackRightLift;

    tEffectivePivotServoIndex = ((FRONT_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFrontRightPivot = 180 - aFrontRightPivot;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aFrontRightPivot;
    sServoNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aFrontRightLift;

    if (aDoMove) {
        synchronizeMoveAllServosAndCheckInputAndWait();
    }
}

/*
 * A subset of the functionality of transformAndSetAllServos() -> less arguments needed :-)
 */
void transformAndSetPivotServos(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot, int aFrontRightPivot,
        uint8_t aDirection, bool doMirror, bool aDoMove) {
    uint8_t tIndexToAdd = aDirection * SERVOS_PER_LEG;
    uint8_t tXorToGetMirroredIndex = 0x0;
// Invert angles for pivot servos
    bool doInvert = false;
    if (doMirror) {
// XOR the index with this value to get the mirrored index
        tXorToGetMirroredIndex = getMirrorXorMask(aDirection);
        doInvert = true;
    }

    uint8_t tEffectivePivotServoIndex;
    tEffectivePivotServoIndex = ((FRONT_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFrontLeftPivot = 180 - aFrontLeftPivot;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aFrontLeftPivot;

    tEffectivePivotServoIndex = ((BACK_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBackLeftPivot = 180 - aBackLeftPivot;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aBackLeftPivot;

    tEffectivePivotServoIndex = ((BACK_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBackRightPivot = 180 - aBackRightPivot;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aBackRightPivot;

    tEffectivePivotServoIndex = ((FRONT_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFrontRightPivot = 180 - aFrontRightPivot;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aFrontRightPivot;

    if (aDoMove) {
        synchronizeMoveAllServosAndCheckInputAndWait();
    }
}

/*
 * Transform index of servo by direction and mirroring
 */
uint8_t transformOneServoIndex(uint8_t aServoIndexToTransform, uint8_t aDirection, bool doMirror) {
    uint8_t tXorToGetMirroredIndex = 0x0;
    if (doMirror) {
// XOR the index with this value to get the mirrored index
        tXorToGetMirroredIndex = getMirrorXorMask(aDirection);
    }
    return ((aServoIndexToTransform + (aDirection * SERVOS_PER_LEG)) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
}

void testTransform() {
// left legs are close together, right legs are in straight right direction
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_FORWARD, false, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_FORWARD, true, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_BACKWARD, false, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_BACKWARD, true, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_LEFT, false, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_LEFT, true, false);
    printArrayPositions(&Serial);
}

void setPivotServos(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot, int aFrontRightPivot) {
    sServoNextPositionArray[FRONT_LEFT_PIVOT] = aFrontLeftPivot;
    sServoNextPositionArray[BACK_LEFT_PIVOT] = aBackLeftPivot;
    sServoNextPositionArray[BACK_RIGHT_PIVOT] = aBackRightPivot;
    sServoNextPositionArray[FRONT_RIGHT_PIVOT] = aFrontRightPivot;
    synchronizeMoveAllServosAndCheckInputAndWait();
}

/*
 * Accepts height from 0 to 100
 */
void setLiftServoHeight(ServoEasing & aLiftServo, uint8_t aHeightPercent) {
    if (aHeightPercent > 100) {
        aHeightPercent = 100;
    }
    int tDegreeForLiftServo = map(aHeightPercent, 0, 100, LIFT_MAX_ANGLE, LIFT_MIN_ANGLE);
    aLiftServo.easeTo(tDegreeForLiftServo);
}

/*
 * Set all servos to the same angle
 */
void setLiftServos(int aBodyHeightAngle) {
    sServoNextPositionArray[FRONT_LEFT_LIFT] = aBodyHeightAngle;
    sServoNextPositionArray[BACK_LEFT_LIFT] = aBodyHeightAngle;
    sServoNextPositionArray[BACK_RIGHT_LIFT] = aBodyHeightAngle;
    sServoNextPositionArray[FRONT_RIGHT_LIFT] = aBodyHeightAngle;
    synchronizeMoveAllServosAndCheckInputAndWait();
}

void setLiftServos(int aFrontLeftLift, int aBackLeftLift, int aBackRightLift, int aFrontRightLift) {
    sServoNextPositionArray[FRONT_LEFT_LIFT] = aFrontLeftLift;
    sServoNextPositionArray[BACK_LEFT_LIFT] = aBackLeftLift;
    sServoNextPositionArray[BACK_RIGHT_LIFT] = aBackRightLift;
    sServoNextPositionArray[FRONT_RIGHT_LIFT] = aFrontRightLift;
    synchronizeMoveAllServosAndCheckInputAndWait();
}

/*
 * Used after change of sBodyHeightAngle
 */
void setLiftServosToBodyHeight() {
    // Set values direct, since we expect only a change of 2 degree
    for (uint8_t i = LIFT_SERVO_OFFSET; i < NUMBER_OF_SERVOS; i += SERVOS_PER_LEG) {
        sServoArray[i]->write(sBodyHeightAngle);
    }
}

void setAllServos(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot, int aFrontRightPivot, int aFrontLeftLift,
        int aBackLeftLift, int aBackRightLift, int aFrontRightLift) {
    sServoNextPositionArray[FRONT_LEFT_PIVOT] = aFrontLeftPivot;
    sServoNextPositionArray[BACK_LEFT_PIVOT] = aBackLeftPivot;
    sServoNextPositionArray[BACK_RIGHT_PIVOT] = aBackRightPivot;
    sServoNextPositionArray[FRONT_RIGHT_PIVOT] = aFrontRightPivot;

    sServoNextPositionArray[FRONT_LEFT_LIFT] = aFrontLeftLift;
    sServoNextPositionArray[BACK_LEFT_LIFT] = aBackLeftLift;
    sServoNextPositionArray[BACK_RIGHT_LIFT] = aBackRightLift;
    sServoNextPositionArray[FRONT_RIGHT_LIFT] = aFrontRightLift;
    synchronizeMoveAllServosAndCheckInputAndWait();
}

void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, int aDegree) {
    moveOneServoAndCheckInputAndWait(aServoIndex, aDegree, sServoSpeed);
}

void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, int aDegree, uint16_t aDegreesPerSecond) {
    sServoArray[aServoIndex]->startEaseTo(aDegree, aDegreesPerSecond, false);
    do {
        if (delayAndCheck(REFRESH_INTERVAL / 1000)) { // 20 ms - REFRESH_INTERVAL is in Microseconds
            return;
        }
    } while (!sServoArray[aServoIndex]->update());
}

void updateAndCheckInputAndWaitForAllServosToStop() {
    do {
        if (delayAndCheck(REFRESH_INTERVAL / 1000)) { // 20 ms - REFRESH_INTERVAL is in Microseconds
            return;
        }
    } while (!updateAllServos());
}

void synchronizeMoveAllServosAndCheckInputAndWait() {
    setEaseToForAllServos();
    synchronizeAllServosAndStartInterrupt(false);
    updateAndCheckInputAndWaitForAllServosToStop();
}
