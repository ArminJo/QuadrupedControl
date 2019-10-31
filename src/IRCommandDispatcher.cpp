/*
 * IRCommandDispatcher.cpp
 *
 * Receives command by IR and calls functions specified in a mapping array.
 *
 * Program for controlling a mePed Robot V2 with 8 servos using an IR Remote at pin A0
 * Supported IR remote are KEYES (the original mePed remote) and WM10
 * Select the one you have at line 20 in IRCommandMapping.h
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

#include "IRCommandDispatcher.h"
#include <IRLremote.h>      // include IR Remote library

#include "IRCommandMapping.h"

//#define INFO // comment this out to see serial info output
//#define WARN // comment this out to see serial info output
#ifdef INFO
#define WARN
#endif

#if (IR_CONTROL_CODING == 'P')
CPanasonic IRLremote;
#else
CNec IRLremote;
#endif

bool sJustCalledExclusiveIRCommand = false;
bool sExecutingExclusiveCommand = false;        // set if we just execute a command by dispatcher
uint8_t sRejectedExclusiveCommand = COMMAND_INVALID; // Storage for rejected command to allow the actual command to end, before it is called by main loop.
uint8_t sCurrentCommandCalled = COMMAND_INVALID; // The code for the current called command
bool sCurrentCommandIsRepeat;
bool sAtLeastOneValidIRCodeReceived = false; // one time flag. Set if we received a valid IR code. Used for breaking timeout for auto move.
uint32_t sLastTimeOfIRCodeReceived; // millis of last IR command
/*
 * Flag for movements to stop, set by checkIRInputForNonExclusiveCommand().
 * It works like an exception so we do not need to propagate the return value from the delay up to the movements.
 * Instead we can use "if (sRequestToStopReceived) return;" (available as macro RETURN_IF_STOP).
 */
bool sRequestToStopReceived;

void setupIRDispatcher() {

    // Start reading the remote. PinInterrupt or PinChangeInterrupt* will automatically be selected
    if (!IRLremote.begin(IR_RECEIVER_PIN)) {
#ifdef INFO
        Serial.println(F("You did not choose a valid pin"));
#endif
    }
}

/*
 * Handling of sRequestToStopReceived
 * @return see checkAndCallCommand()
 */
bool loopIRDispatcher(bool takeRejectedCommand) {

    // here we are stopped, so lets start a new turn
    sRequestToStopReceived = false;

    /*
     * search IR code or take last rejected command and call associated function
     */
    uint8_t tIRCode;
    if (takeRejectedCommand && (sRejectedExclusiveCommand != COMMAND_INVALID)) {
#ifdef INFO
        Serial.print(F("Take rejected command = 0x"));
        Serial.println(sRejectedExclusiveCommand, HEX);
#endif
        tIRCode = sRejectedExclusiveCommand;
        sRejectedExclusiveCommand = COMMAND_INVALID;
    } else {
        tIRCode = getIRCommand(false);
    }
    return checkAndCallCommand(tIRCode);
}

/*
 * Does repeat detection and sets sCurrentCommandIsRepeat accordingly
 * Wait for next IR command, and return last IR code (instead of repeat code) if repeat detected.
 */
uint8_t getIRCommand(bool doWait) {
    static uint8_t sLastIRValue = COMMAND_EMPTY; // for repeat detection
//    static uint16_t sReferenceAddress = 0; // store first received address here for better IR-receive error handling

    uint8_t tIRReturnValue = COMMAND_EMPTY;

    do {
        if (IRLremote.available()) {
            // Get the new data from the remote
            auto tIRData = IRLremote.read();

            if ((tIRData.address == IR_REPEAT_ADDRESS && tIRData.command == IR_REPEAT_CODE)
                    || (sLastIRValue == tIRData.command && sLastTimeOfIRCodeReceived > millis() - IR_REPEAT_TIMEOUT_MS)) {
                sLastTimeOfIRCodeReceived = millis();

                /*
                 * Handle repeat(ed) code
                 */
#ifdef INFO
                Serial.println(F("Repeat received"));
#endif
                if (sLastIRValue != COMMAND_EMPTY) {
                    sCurrentCommandIsRepeat = true;
                    tIRReturnValue = sLastIRValue;
                }
            } else {
                sLastTimeOfIRCodeReceived = millis();

                /*
                 * Regular code here
                 */
#ifdef INFO
                Serial.print(F("A=0x"));
                Serial.print(tIRData.address, HEX);
                Serial.print(F(" C=0x"));
                Serial.print(tIRData.command, HEX);
#endif

                tIRReturnValue = tIRData.command;
//              if (sReferenceAddress == 0) {
//                  // store reference address for error detection
//                  sReferenceAddress = tIRData.address;
//              }
                if (tIRData.address == IR_ADDRESS) {
                    // Received new code (with right address)
                    sCurrentCommandIsRepeat = false;
                    sLastIRValue = tIRReturnValue;
#ifdef INFO
                    Serial.println();
#endif
                    break;
                } else {
#ifdef INFO
                    Serial.println(F(" Unknown"));
#endif
                    // unknown code - maybe here, because other interrupts interfere with the IR Interrupt
                    // Disable repeat in order not to repeat the wrong command
                    sLastIRValue = COMMAND_EMPTY;
                }
            }
        }
    } while (doWait);

    return tIRReturnValue;
}

/*
 * Return values of checkAndCallCommand
 */
#define CALLED 0
#define IR_CODE_EMPTY 1
#define NOT_CALLED_MASK 0x02
#define FOUND_BUT_EXCLUSIVE_LOCK 2
#define FOUND_BUT_REPEAT_NOT_ACCEPTED 3
#define NOT_FOUND_MASK 0x04
#define IR_CODE_NOT_FOUND 4
/*
 * Sets sJustCalledExclusiveIRCommand, sExecutingExclusiveCommand
 */
uint8_t checkAndCallCommand(uint8_t aIRCode) {
    if (aIRCode == COMMAND_EMPTY) {
        return IR_CODE_EMPTY;
    }

    for (uint8_t i = 0; i < sizeof(IRMapping) / sizeof(struct IRToCommandMapping); ++i) {
        if (aIRCode == IRMapping[i].IRCode) {

#ifdef INFO
            Serial.print(F("Found command "));
            Serial.println(reinterpret_cast<const __FlashStringHelper *>(IRMapping[i].CommandString));
#endif

            // one time flag used for breaking timeout for auto move.
            sAtLeastOneValidIRCodeReceived = true;

            /*
             * Check for repeat and call the function specified in IR mapping
             */
            if (sCurrentCommandIsRepeat && !(IRMapping[i].Flags & IR_COMMAND_FLAG_ACCEPT_REPEAT)) {
                return FOUND_BUT_REPEAT_NOT_ACCEPTED;
            }

            /*
             * Do not accept recursive call of the same command
             */
            if (sCurrentCommandCalled == aIRCode) {
#ifdef WARN
                Serial.print(F("Recursive command 0x"));
                Serial.print(sRejectedExclusiveCommand, HEX);
                Serial.println(" not accepted");
#endif
                return FOUND_BUT_REPEAT_NOT_ACCEPTED;
            }

            bool tIsExclusiveCommand = !(IRMapping[i].Flags & IR_COMMAND_FLAG_NOT_EXCLUSIVE);
            if (tIsExclusiveCommand) {
                if (sExecutingExclusiveCommand) {
                    sRejectedExclusiveCommand = aIRCode;
                    return FOUND_BUT_EXCLUSIVE_LOCK;
                }
                /*
                 * Cannot store just flags, since they may be overwritten by a recursive call from the blocking command execution below.
                 */
                sJustCalledExclusiveIRCommand = true;
                sExecutingExclusiveCommand = true;
            }
            sCurrentCommandCalled = aIRCode;
            IRMapping[i].CommandToCall(); // This call may be blocking!!!
            sExecutingExclusiveCommand = false;
            sCurrentCommandCalled = COMMAND_INVALID;
            return CALLED;
        }
    }
    return IR_CODE_NOT_FOUND;
}

/*
 * @return  true (and sets sRequestToStopReceived)  - if invalid IR command received
 */
bool checkIRInputForNonExclusiveCommand() {

    uint8_t tCheckResult = checkAndCallCommand(getIRCommand(false));
    if ((tCheckResult & NOT_FOUND_MASK) || (tCheckResult == FOUND_BUT_EXCLUSIVE_LOCK)) {
        /*
         * IR command not found in mapping or another exclusive command -> request stop
         */
        sRequestToStopReceived = true; // return to loop
        return true;
    }
// Command found in mapping
    return false;
}

void printIRCommandString(uint8_t aIRCode) {
#ifdef INFO
    Serial.print(F("IRCommand="));
    for (uint8_t i = 0; i < sizeof(IRMapping) / sizeof(struct IRToCommandMapping); ++i) {
        if (aIRCode == IRMapping[i].IRCode) {
            Serial.println(reinterpret_cast<const __FlashStringHelper *>(IRMapping[i].CommandString));
            return;
        }
    }
    Serial.println(reinterpret_cast<const __FlashStringHelper *>(unknown));
#endif
}

