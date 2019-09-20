/*
 * QuadrupedControl.cpp
 *
 * Program for controlling a mePed Robot V2 with 8 servos.
 * The IR Remote is attached at pin A0. Supported IR remote are:
 *          KEYES (the original mePed remote)
 *          KEYES_CLONE (the one with numberpad and direction control swapped, which you get when you buy a KEYES at aliexpress).
 *          WM10
 * If you use another than the KEYES_CLONE, you have to select the one you use at line 20ff. in IRCommandMapping.h
 *
 * To run this example need to install the "ServoEasing", "IRLremote" and "PinChangeInterrupt" libraries under "Tools -> Manage Libraries..." or "Ctrl+Shift+I"
 * Use "ServoEasing", "IRLremote" and "PinChangeInterrupt" as filter string.
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

#include "QuadrupedServoControl.h"
#include "QuadrupedMovements.h"
#include "IRCommandDispatcher.h"  // for loopIRDispatcher
#include "IRCommandMapping.h" // for IR_REMOTE_NAME

#define VERSION_EXAMPLE "3.0"

/*
 * Create your own basic movement here
 *
 * Servos available:
 *  frontLeftPivotServo, frontLeftLiftServo
 *  backLeftPivotServo, backLeftLiftServo
 *  backRightPivotServo, backRightLiftServo
 *  frontRightPivotServo, frontRightLiftServo
 *
 * Example commands:
 * delayAndCheckIRInput(1000);
 * frontLeftLiftServo.easeTo(LIFT_MIN_ANGLE);
 * setLiftServos(LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE);
 * setPivotServos(100, 100, 80, 80);
 */
void doTest() {
	frontLeftLiftServo.write(90);
	backRightLiftServo.easeTo(90);
}

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
}

void loop() {
	/*
	 * Check for IR commands and execute them.
	 * Returns only AFTER finishing of requested movement
	 */
	loopIRDispatcher();
}
