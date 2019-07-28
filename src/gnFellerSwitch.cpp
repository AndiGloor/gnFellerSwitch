/*  GN FellerSwitch Library
 *  =======================
 *  
 *  Library for Feller Switches "Feller EDIZIOdue Elektroniktaster (900-3928.FMI.L.61)".
 *  See Readme for details.
 *  
 *  Tested with Arduino UNO and gnFellerTaster PCB (compatible with Arduino Uno).
 *  
 *  2019-07-28	V1.3.2    Andreas Gloor            Bugfix LedDisplay: Not showing the Leds in case of rare Led-Updates
 *  2019-07-21	V1.3.1    Andreas Gloor            Implements: LedDisplay
 *  2018-09-08  V1.1.2    Andreas Gloor            Reset ErrorCount after Init
 *  2018-07-21  V1.1.1    Andreas Gloor            Initial Version
 *  
 *  Feller and EDIZIOdue are Trademarks of Feller AG, http://www.feller.ch
 *  
 *  MIT License
 *  
 *  Copyright (c) 2018-19 Andreas Gloor
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */


// Include necessary Library's
#include "gnFellerSwitch.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <Stream.h>



// Public ////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Constructor and Begin ---------------------------------------------------------------------------------------------
// Constructor
gnFellerSwitch::gnFellerSwitch(HardwareSerial& device) {
	_hwStream = &device;	
}

gnFellerSwitch::gnFellerSwitch(SoftwareSerial& device) {
	_swStream = &device;	
}



// Begin - Initializes the communication
bool gnFellerSwitch::begin(uint32_t baudRate, uint32_t initialBaudRate) {
	#ifdef GNFS_DEBUG_DIGITAL																							// Initialize the Pins for Digital-Debugging if defined
		pinMode(GNFS_DEBUG_DIGITAL_PIN_ERR, OUTPUT);																	// Attach a Logic-Analyzer to the pins to understand timing-critical errors
		pinMode(GNFS_DEBUG_DIGITAL_PIN_ACT, OUTPUT);																	// Pins are defined in the header-file
		pinMode(GNFS_DEBUG_DIGITAL_PIN_DET, OUTPUT);
		digitalWrite(GNFS_DEBUG_DIGITAL_PIN_ERR, LOW);
		digitalWrite(GNFS_DEBUG_DIGITAL_PIN_ACT, LOW);
		digitalWrite(GNFS_DEBUG_DIGITAL_PIN_DET, LOW);
	#endif
	
	if (_getFellerBaudrateFlag(baudRate) == 0xFF || 																	// Validate Baudrates
			_getFellerBaudrateFlag(initialBaudRate) == 0xFF) {
		
		_logError(errInitInvalidBaudrate);																				// Save the occurred error and fail
		return false;
	}
	
	_beginSerial(baudRate);																								// Step1: Try Reset on desired Baudrate (if we where allready initialized bevore boot) - Ignore errors
	delay(250);																											// Get the Switch time to sense the Begin
	_tryReset();
	_logError(errNone);
	
	_beginSerial(initialBaudRate);																						// Step2: Reset on initial Baudrate (Hardware-configured Baudrate)
	delay(250);																											// Get the Switch time to sense the Begin
	if (!_tryReset()) {
		_logError(errInitResetFellerSwitchFailed);																		// Save the occurred error (regardless of Timeout or Verify-Error, set a InitResetFailed)
		return false;
	}
	
	_txBuffer[0] = 0x00;																								// Step3: Initiate all protocol settings including - Default: asynchronous without handshake
	_txBuffer[1] = _getFellerBaudrateFlag(baudRate);
	_txBuffer[2] = 0x00;																								// Auto-Timeout
	_txBuffer[3] = 0b00000001;																							// Enable ButtonState.indication
	_txBuffer[4] = 0b00000001;																							// Enable SystemState.indication
	if (!_sendFrame(setSystemSettings_request, 5, setSystemSettings_confirm)) {
		_logError(errInitSetSystemSetingsRequestFailed);																// Save the occurred error and fail
		return false;
	}
	_beginSerial(baudRate);																								// Step4: Switch to desired Baudrate
	
	if (!_sendFrame(getSystemInfo_request, 0, getSystemInfo_confirm)) {													// Step5: Get System Informations from Switch
		_logError(errInitGetSystemInfoFailed);																			// Save the occurred error and fail
		return false;
	}
	if (_rxBuffer[0] < 0x10) {																							// Test if we have at least Firmware V1.0
		_logError(errInitSwitchFirmwareToLow);																			// Save the occurred error and fail
		return false;
	}
	_switchFirmwareVersion = _rxBuffer[0];
	if (_rxBuffer[1] > 0x03) {																							// Test if we have a known hardware-version
		_logError(errInitSwitchHardwareUnknown);																		// Save the occurred error and fail
		return false;
	}
	switch (_rxBuffer[1]) {																								// Save the values of Push-Buttons and LED's
		case 0x00:
			_switchPushButtonCount = 4;
			_switchLedCount = 0;
			break;
		case 0x01:
			_switchPushButtonCount = 8;
			_switchLedCount = 0;
			break;
		case 0x02:
			_switchPushButtonCount = 4;
			_switchLedCount = 6;
			break;
		case 0x03:
			_switchPushButtonCount = 8;
			_switchLedCount = 8;
			break;
	}
	
	_setFlag(GNFS_FLAG_Initialized);																					// Store the successful init and return ok
	_errorCount = 0;
	return true;
}



// Communication Handling --------------------------------------------------------------------------------------------
// HandleCommunication - Writes all Changes to the Switch (if necessary) and processes any inputs from the switch
void gnFellerSwitch::handleCommunication() {
	if (!_readFlag(GNFS_FLAG_Initialized)) {																			// Do nothing until initialized
		_logError(errNotInitializedOrInitFailed);
		return;
	}
	
	_receiveFrame(gnFellerSwitch::receiveAndCallback);																	// Receive a Frame if something available
	
	if (_readFlag(GNFS_FLAG_LedDisplayMode_ButtonPressed_TimerActive)) {												// Check Timer if necessary
		_updateLedDisplayMode();
	}

	if (_readFlag(GNFS_FLAG_RequestUpdateForLedBrightness)) {															// Update LED Brightness
		_updateLedBrightness();
		_clearFlag(GNFS_FLAG_RequestUpdateForLedBrightness);
	}
	
	if (_readFlag(GNFS_FLAG_RequestUpdateForLedState)) {																// Update LED Color & blinking
		_updateLedState();
		_clearFlag(GNFS_FLAG_RequestUpdateForLedState);
	}
}



// Services ----------------------------------------------------------------------------------------------------------
// AttachRawButtonHandler - Attaches a Callback-Handler for the Button-Events
bool gnFellerSwitch::attachButtonHandler(rawButtonHandlerCallback rawButtonHandler) {
	_rawButtonHandler = rawButtonHandler;
	_setFlag(GNFS_FLAG_ButtonHandler_Raw_Enabled);
}



// AttachSingleButtonHandler - Attaches a Callback-Handler for the Button-Events
bool gnFellerSwitch::attachButtonHandler(singleButtonHandlerCallback singleButtonHandler) {
	_singleButtonHandler = singleButtonHandler;
	_setFlag(GNFS_FLAG_ButtonHandler_Single_Enabled);
}



// GetButtonState - Gets the RAW press-state of the Buttons
uint8_t gnFellerSwitch::getButtonState() {
	return _buttonState;
}
// GetButtonState - Gets the press-state of a single Button
bool gnFellerSwitch::getButtonState(buttonPosition buttonNumber) {
	return bitRead(_buttonState, (buttonNumber - 1));
}



// GetLedColor - Gets the Color of a single Led
gnFellerSwitch::ledColor gnFellerSwitch::getLedColor(ledPosition ledNumber) {
	if (bitRead(_ledState[0], ledNumber) || 
			bitRead(_ledState[3], ledNumber)) {
		return red;
	} else if (bitRead(_ledState[1], ledNumber) || 
						 bitRead(_ledState[4], ledNumber)) {
		return green;
	} else {
		return blue;
	}
}



// GetLedState - Gets the State of a single Led
gnFellerSwitch::ledState gnFellerSwitch::getLedState(ledPosition ledNumber) {
	if (bitRead(_ledState[0], ledNumber) || 
			bitRead(_ledState[1], ledNumber) || 
			bitRead(_ledState[2], ledNumber)) {
		return on;
	} else if (bitRead(_ledState[3], ledNumber) || 
						 bitRead(_ledState[4], ledNumber) ||
						 bitRead(_ledState[5], ledNumber)) {
		return blinking;
	} else {
		return off;
	}
}



// SetLed - Switch a LED of the switch to on/off/blinking - Don't forget to call handleCommunication after change!
void gnFellerSwitch::setLed(gnFellerSwitch::ledPosition pos, gnFellerSwitch::ledState state, gnFellerSwitch::ledColor color) {
	if (!_readFlag(GNFS_FLAG_Initialized)) {																			// Do nothing until initialized
		_logError(errNotInitializedOrInitFailed);
		return;
	}
	
	uint8_t bitMask = state & color;																					// Bitwise and, produces rgbrgb blinking/on Mask.
	uint8_t oldState;
	
	for (uint8_t i = 0; i < 6; i++) {																					// _ledState stores the Values as Feller expect them: [0] = red, on; [5] = blue, blinking;
		oldState = _ledState[i];
		bitWrite(_ledState[i], pos, bitRead(bitMask, i));																// Apply the value of the rgbrgb mask
		if (oldState != _ledState[i]) {																					// Set Flag if something changed
			_setFlag(GNFS_FLAG_RequestUpdateForLedState);
		}
	}
}



// SetLedBrightness - Sets the Brightness of all LED's simultaniously - Caution: 0 = all LED's off! - Don't forget to call handleCommunication after change!
void gnFellerSwitch::setLedBrightness(uint8_t brightness) {
	if (!_readFlag(GNFS_FLAG_Initialized)) {																			// Do nothing until initialized
		_logError(errNotInitializedOrInitFailed);
		return;
	}
	
	if (_brightness == brightness) {																					// Do nothing if brightness is already on the desired value
		return;
	}
	
	_brightness = brightness;																							// Store the new Value and set update Flag
	_setFlag(GNFS_FLAG_RequestUpdateForLedBrightness);
}



// SetLedDisplayMode - Sets the DisplayMode for the LED's. Enables you to dark out the whole panel at night.
void gnFellerSwitch::setLedDisplayMode(ledDisplayMode displayMode, uint8_t onSeconds) {
  	if (displayMode != _ledDisplayMode) {																				// Set Flag for Led-Update if something changed
		_setFlag(GNFS_FLAG_RequestUpdateForLedState);
	}
	_ledDisplayMode = displayMode;																						// Store the new values
	if (displayMode != full) {
		_ledDisplayModeOnSeconds = onSeconds;
	} else {
		_ledDisplayModeOnSeconds = 0;
	}
}



// Error Handling ----------------------------------------------------------------------------------------------------
// GetErrorCounter - Returns the count of errors occurred since last query
uint8_t gnFellerSwitch::getErrorCounter() {
	uint8_t returnValue = _errorCount;
	_errorCount = 0;
	return returnValue;
}



// GetLastError - Returns the last Error and reset the error-store
gnFellerSwitch::gnFellerSwitchError gnFellerSwitch::getLastError() {
	gnFellerSwitchError returnValue = _lastError;
	_lastError = errNone;
	return returnValue;
}



// GetLastSwitchControllerError - Returns the last Error occurred on the Feller-Switch side and reset the error-store
uint8_t	gnFellerSwitch::getLastSwitchControllerError() {
	uint8_t returnValue = _lastErrorSwitchController;
	_lastErrorSwitchController = 0;
	return returnValue;
}



// GetSwitchControllerErrorCounter - Returns the count of errors occurred on the Feller-Switch since last query
uint8_t gnFellerSwitch::getSwitchControllerErrorCounter() {
	uint8_t returnValue = _errorCountSwitchController;
	_errorCountSwitchController = 0;
	return returnValue;
}



// Private ///////////////////////////////////////////////////////////////////////////////////////////////////////////



// Constructor and Begin ---------------------------------------------------------------------------------------------
// GetFellerBaudrateFlag - Translates a Baudrate into a Byte, as requested in Service SetSystemSettings.request
uint8_t gnFellerSwitch::_getFellerBaudrateFlag(uint32_t baudRate) {
	switch (baudRate) {
		case 1200:		return 0x01;
		case 2400:		return 0x02;
		case 4800:		return 0x04;
		case 9600:		return 0x08;
		case 19200:		return 0x10;
		case 38400:		return 0x20;
		case 57600:		return 0x30;
		case 115200:	return 0x60;
		default:		return 0xFF;
	}
}



// Communication Handling --------------------------------------------------------------------------------------------
// BeginCtsPause - Initializes the Timer to assure we have the go to send (clear to send), respecting the minimum-framegap-rule
void gnFellerSwitch::_beginCtsPause() {
	_ctsTimer = millis();
}



// BeginSerial - Invoke the Begin-Command of the corresponding (Soft-)Serial Class
void gnFellerSwitch::_beginSerial(uint32_t baudRate) {
	_baudRate = baudRate;																								// Store Baudrate (needed for timeout calculations)
	
	if (_hwStream) {																									// Call Begin Function of corresponding SerialObject...
    _hwStream->begin(baudRate);
  }
  else {
    _swStream->begin(baudRate);
  }
	
	while (!_stream) {																									// Safe Stream from SerialObject while it is ready
		_stream = !_hwStream? (Stream*)_swStream : _hwStream;
	}
	
	_stream->setTimeout(_calculateInterByteTimeout());																	// Calculate Inter-Byte-Timeout, corresponding to the Baudrate and apply.
}



// BitCount - Gets the count of Bits set to 1
uint8_t gnFellerSwitch::_bitCount(uint8_t bit) {
	uint8_t returnValue = 0;
	for (uint8_t i = 0; i < 7; i++) {																					// Iterate each bit
		returnValue += bitRead(bit, i);
	}
	return returnValue;
}



// CalculateFrameHeader - Calculates a Frameheader as defined by Feller
uint8_t gnFellerSwitch::_calculateFrameHeader(uint8_t payloadBytesCount, _fellerService service) {
	if (service != noServiceOrReset) {																					// Include 1Byte for ServiceId, unless no Service specified
		payloadBytesCount++;
	}
	uint8_t returnValue = payloadBytesCount & 0b00011111;																// Length-Bits
	returnValue |= 0b00100000;																							// Fixed Values
	returnValue |= (_bitCount(returnValue) % 2) << 7;																	// Parity-Bit used to get all bits even; so we can check with Modulo 2
	return returnValue;
}



// CheckCts - Checks for the CTS-Timer, respecting the minimum-framegap-rule - return true if clear to send
bool gnFellerSwitch::_checkCts() {
	// Its not clearly specified how long we have to wait
	return (millis() - _ctsTimer > (_calculateInterByteTimeout() * 12 / 10));											// Its not clearly specified how long we have to wait
																														// According to the Feller Documentation it needs at least a interbyte time, on top i added 20% safety-margin
}



// CheckFrameHeaderParity - Validate if a Frameheader-Byte has correct checksum
bool gnFellerSwitch::_checkFrameHeaderParity(uint8_t frameHeader) {
	return (_bitCount(frameHeader) % 2 == bitRead(frameHeader, 7));														// Parity-Bit used to get all bits even; so we can check with Modulo 2
}



// HandleIndicationFrame - Process a unexpected Frame from the Switch - shoud be a indication Frame (Button or Error)
void gnFellerSwitch::_handleIndicationFrame(uint8_t indicationService) {
	#ifdef GNFS_DEBUG_SERIAL
		GNFS_DEBUG_SERIAL_PORT.print(F("Handler called for Indication-Frame "));
		_debugPrintHex(indicationService);
		GNFS_DEBUG_SERIAL_PORT.println();
	#endif
	
	switch (indicationService) {
		case buttonState_indication:
			if (_ledDisplayModeOnSeconds != 0) {																		// Reset the LedDisplayMode Timer and set Flags (if any non-full-mode enabled)
				_setFlag(GNFS_FLAG_LedDisplayMode_ButtonPressed_TimerActive);											// Set the Flag for active DisplayMode Timer
				_setFlag(GNFS_FLAG_RequestUpdateForLedState);															// Set the Flag for LED Update
				_ledDisplayModeTimer = millis();
			}
			if (_readFlag(GNFS_FLAG_ButtonHandler_Raw_Enabled)) {														// Invoke the RAW-Button-Handler callback (if attached)
				_rawButtonHandler(_rxBuffer[0]);
			}
			if (_readFlag(GNFS_FLAG_ButtonHandler_Single_Enabled)) {													// Invoke the SINGLE-Button-Handler callback (if attached)
				for (uint8_t i = 0; i < 8; i++) {																		// Iterate each bit and call the handler if changed
					if (bitRead(_rxBuffer[0], i) != bitRead(_buttonState, i)) {
						_singleButtonHandler((i + 1), bitRead(_rxBuffer[0], i));
					}
				}
			}
			_buttonState = _rxBuffer[0];																				// Save the new ButtonState
			break;
			
		case systemState_indication:																					// Log Feller-SwitchControllerError and continue
			_logSwitchControllerError(_rxBuffer[0]);
			break;
			
		default:																										// Log Error and continue
			_logError(errUnexpectedIndicationFrameReceived);
			break;
	}
}



// ReceiveFrame - Gets a full Frame from Feller-Switch and performs Action as defined with receiveAction
bool gnFellerSwitch::_receiveFrame(_receiveAction action, _fellerService exceptedService) {
	uint8_t buffer[1];
	uint32_t timestamp;
	uint16_t timeout;
	int8_t frameLength = -1;
	uint8_t serviceId;
	
	if (_rxBufferPayloadCount != 0) {																					// Initialize the Buffer and its Counter when needed
		memset(_rxBuffer, 0xEE, GNFS_RX_BUFFERSIZE);
		_rxBufferPayloadCount = 0;
	}
	
	// Check for available Data and handle Timeout
	if (action != demandReset) {																						// Calculate the Timeout
		timeout = _calculateInterFrameTimeout();
	} else {
		timeout = 500;																									// For a Reset, the Switch needs approximately 250ms, regarding to the DataSheet.
	}
	if (!_stream->available() && action == receiveAndCallback) {														// ReceiveAndCallback only do something if Data available as the others wait for data (or timeout)
		return true;
	} else {
		timestamp = millis();																							// Start Timer
		while (!_stream->available() && millis() - timestamp < (timeout)) {												// Blocking wait till data arrive or timeout
			delayMicroseconds(10);
		}
		if (!_stream->available()) {
			_logError(errTimeoutWaitingForFrameFromFellerSwitch);														// Store the error and return Failure
			return false;
		}
	}
	
	// Process Frame-Header
	while (frameLength < 0) {
		if (_stream->readBytes(buffer, 1) == 0) {																		// Read the first Byte and check for timeout if no Data arrive (during second turn)
			if (!(millis() - timestamp < (timeout))) {																	// If read timed out, wait check if a Frame-Timeout is also past (out of spec) and then finally give up
				_logError(errTimeoutWaitingForFrameHeaderFromFellerSwitch);
				return false;
			}
		} else {
			if ((buffer[0] & 0b01110000) == 0b00100000 &&																// Validate if we got a valid Frame-Header and store the Frame-Length (payload, without header)
					(buffer[0] & 0b00001111) <= GNFS_TX_BUFFERSIZE + 1 &&
					_checkFrameHeaderParity(buffer[0])) {
				frameLength = (buffer[0] & 0b00001111);
			} else {																									// Store the Error of a unexpected or invalid Byte received
				_logError(errUnexpectedOrCorruptedByteReceivedWhileFrameHeaderExpected);								// Note: this error can caused by SoftSerial
				timestamp = millis();																					// Start Timer and wait max. a Frame-Timeout
			}
		}
	}
	
	#ifdef GNFS_DEBUG_SERIAL																							// Debug Code
		GNFS_DEBUG_SERIAL_PORT.print(F("<<< ["));
		GNFS_DEBUG_SERIAL_PORT.print(frameLength + 1);
		GNFS_DEBUG_SERIAL_PORT.print(F("]\t"));
		_debugPrintHex(buffer[0]);
	#endif
	
	// Process Frame
	for (uint8_t readedBytes = 0; readedBytes < frameLength; readedBytes++) {
		if (_stream->readBytes(buffer, 1) == 1) {																		// Receive requested Bytes (or Timeout)
			if (readedBytes == 0)	{																					// Store the first Byte as ServiceId, the others in _rxBuffer
				serviceId = buffer[0];
			} else {
				_rxBuffer[readedBytes - 1] = buffer[0];
			}
		} else {
			_logError(errTimeoutWaitingForNextByteFromFellerSwitch);													// Store the error and fail
			#ifdef GNFS_DEBUG_SERIAL
				GNFS_DEBUG_SERIAL_PORT.print(F("TIMEOUT IN ReadFrame @Byte:"));
				GNFS_DEBUG_SERIAL_PORT.println(readedBytes + 1);
			#endif
			return false;
		}
	}
	_beginCtsPause();																									// Start CTS-Timer -> respect inter-frame space before sending a frame
	
	#ifdef GNFS_DEBUG_SERIAL																							// Debug Code
		GNFS_DEBUG_SERIAL_PORT.print(F("\t"));
		_debugPrintHex(serviceId);
		for (uint8_t i = 0; i < frameLength - 1; i++) {
			GNFS_DEBUG_SERIAL_PORT.print(F("\t"));
			_debugPrintHex(_rxBuffer[i]);
		}
		GNFS_DEBUG_SERIAL_PORT.println();
	#endif
	
	// Perform Action
	if (frameLength == 0) {																								// A zero-Payload Frameheader indicates a Reset
		#ifdef GNFS_DEBUG_SERIAL
			GNFS_DEBUG_SERIAL_PORT.println(F("ReceiveFrame: ResetCheck"));
		#endif
		if (action == demandSpecificServiceRetryOnce) {																	// Retry if desired or direct return
			return _receiveFrame(demandSpecificServiceFail, exceptedService);
		} else {
			return (action == demandReset);																				// With Reset-Indication, we are only successful if a Reset was requested...
		}
	}
	if (action == receiveAndCallback ||																					// We have to call the Callback-Handler if requested or...
			(action == demandSpecificServiceRetryOnce &&																// ... if action is RetryOnce and not the demanded ServiceId
			 serviceId != exceptedService)) {
		#ifdef GNFS_DEBUG_SERIAL																						// Debug Code
			GNFS_DEBUG_SERIAL_PORT.print(F("ReceiveFrame: Callback Calling for ServiceId "));
			_debugPrintHex(serviceId);
			GNFS_DEBUG_SERIAL_PORT.println();
		#endif

		_handleIndicationFrame(serviceId);																				// Handle the unexpected Frame (shoud be a indication, pushed by the FellerSwitch)
		
		if (action == receiveAndCallback) {																				// Due to requested callback, return true (regardless of possible callback result)
			return true;
		}
		if (action == demandSpecificServiceRetryOnce) {																	// Retry the demand for a specific Service and return result
			#ifdef GNFS_DEBUG_SERIAL
				GNFS_DEBUG_SERIAL_PORT.print(F("ReceiveFrame: Unexpected Service ("));
				_debugPrintHex(serviceId);
				GNFS_DEBUG_SERIAL_PORT.print(F(", got "));
				_debugPrintHex(exceptedService);
				GNFS_DEBUG_SERIAL_PORT.println(F(") give a 2nd Chance"));
			#endif
			return _receiveFrame(demandSpecificServiceFail, exceptedService);
		}
	}
	#ifdef GNFS_DEBUG_SERIAL
		GNFS_DEBUG_SERIAL_PORT.print(F("ReceiveFrame FinalResult: "));
		GNFS_DEBUG_SERIAL_PORT.println((serviceId == exceptedService || exceptedService == noServiceOrReset));
	#endif
	return (serviceId == exceptedService || 																			// Remains the check if a demanded Service received or not - direct return the result of the comparison (noServiceOrReset skips this check)
					exceptedService == noServiceOrReset);
}



// SendFrame - Sends a Command to the Feller-Switch and awaits the Response.
bool gnFellerSwitch::_sendFrame(_fellerService sendService, uint8_t payloadByteCount, _fellerService responseService) {
	#ifdef GNFS_DEBUG_SERIAL
		GNFS_DEBUG_SERIAL_PORT.println();
		GNFS_DEBUG_SERIAL_PORT.print(F(">>> ["));
		GNFS_DEBUG_SERIAL_PORT.print(payloadByteCount + 1 + (sendService != noServiceOrReset));
		GNFS_DEBUG_SERIAL_PORT.print(F("]\t"));
		_debugPrintHex(_calculateFrameHeader(payloadByteCount, sendService));
		if (sendService != noServiceOrReset) {
			GNFS_DEBUG_SERIAL_PORT.print(F("\t"));
			_debugPrintHex(sendService);
		}
		for(uint8_t i = 0; i < payloadByteCount; i++) {
			GNFS_DEBUG_SERIAL_PORT.print(F("\t"));
			_debugPrintHex(_txBuffer[i]);
		}
		GNFS_DEBUG_SERIAL_PORT.println();
		GNFS_DEBUG_SERIAL_PORT.flush();
	#endif

	while (!_checkCts()) {																								// Wait for CTS
		delayMicroseconds(10);
	}
	
	_stream->write(_calculateFrameHeader(payloadByteCount, sendService));												// Send Frame-Header
	if (sendService != noServiceOrReset) {																				// Send Service (unless noServiceOrReset)
		_stream->write(sendService);
	}
	_stream->write(_txBuffer, payloadByteCount);																		// Send requested Bytes
	_stream->flush();																									// Make sure all Bytes are written out
	memset(_txBuffer, 0, GNFS_TX_BUFFERSIZE);																			// Reset Send-Buffer
	if (sendService == noServiceOrReset &&																				// Check if a Reset was sended (0 Payload) 
			payloadByteCount == 0) {
		return _receiveFrame(gnFellerSwitch::demandReset);																// Return Success, corresponding to Frame received
	} else {
		#ifdef GNFS_DEBUG_SERIAL
			GNFS_DEBUG_SERIAL_PORT.print(F("SendFrame: Calling ReceiveFrame (demandSpecificServiceRetryOnce) with "));
			_debugPrintHex(responseService);
			GNFS_DEBUG_SERIAL_PORT.println();
		#endif
		return _receiveFrame(gnFellerSwitch::demandSpecificServiceRetryOnce, responseService);
	}
}



// Services ----------------------------------------------------------------------------------------------------------
// UpdateLedBrightness - Sends the SetLedBrightness.request to the Feller-Switch.
void gnFellerSwitch::_updateLedBrightness() {
	_txBuffer[0] = GNFS_SVC_SetLedBrightness_Variant1;																	// Variant 1: no other available
	_txBuffer[1] = _brightness;

	_sendFrame(setLedBrightness_request, 2, setLedBrightness_confirm);		 											// Send Command and wait for the ok
	if (_rxBuffer[0] != GNFS_SVC_SetLedBrightness_Variant1) {															// Also check for a invalid Response
		_logError(errAnswerFromFellerSwitchInvalid);																	// Store a Invalid-Error
	}
}



// UpdateLedState - Sends the SetLedState.request to the Feller-Switch.
void gnFellerSwitch::_updateLedState() {
	_txBuffer[0] = GNFS_SVC_SetLedState_Variant2;																		// Variant 2: Advanced Frame (inc. blinking)
	for (uint8_t i = 0; i < 6; i++) {																					// Iterate the _ledState Bytes and copy them
		if (_ledDisplayMode == ledDisplayMode::dark 																	// Check for DisplayMode
			&& !_readFlag(GNFS_FLAG_LedDisplayMode_ButtonPressed_TimerActive)) {										// If a Timer is active, we are temporary in regular-Mode
			_txBuffer[1 + i] = 0x0;																						// Dark means no LED to lighten on
		} else if (_ledDisplayMode == ledDisplayMode::one 																// One means only L1 is lighten on (if it shoud) - see bitMask
				   && !_readFlag(GNFS_FLAG_LedDisplayMode_ButtonPressed_TimerActive)) {
			_txBuffer[1 + i] = _ledState[i] & 0b00000001;
		} else {
			_txBuffer[1 + i] = _ledState[i];																			// In any other (normal) Case just copy the byte
		}
	}					
	
	_sendFrame(setLedState_request, 7, setLedState_confirm);	 														// Send Command and wait for the ok
	if (_rxBuffer[0] != GNFS_SVC_SetLedState_Variant2) {																// Also check for a invalid Response
		_logError(errAnswerFromFellerSwitchInvalid);																	// Store a Invalid-Error
	}
}



// UpdateLedDisplayMode - Will be called if Flag "TimerActive" is set; Check if the Timer is running out.
void gnFellerSwitch::_updateLedDisplayMode() {
	if (millis() - _ledDisplayModeTimer > (_ledDisplayModeOnSeconds * 1000) || _ledDisplayModeOnSeconds == 0) {			// Check if Timer is expired (or DisplayMode changed)
		_clearFlag(GNFS_FLAG_LedDisplayMode_ButtonPressed_TimerActive);													// Clear the Flag for an active Timer
		_setFlag(GNFS_FLAG_RequestUpdateForLedState);																	// Set the Flag for LED Update
	}
}



// Error Handling ----------------------------------------------------------------------------------------------------
// DebugMorse - Send short Pulses which can be counted by a Logic-Analyzer
void gnFellerSwitch::_debugMorse(uint8_t pin, uint8_t pulseCount) {
	#if (defined (GNFS_DEBUG_DIGITAL) && defined (GNFS_DEBUG_DIGITAL_ERRORMORSE))
		digitalWrite(pin, HIGH);																						// Begin with a long HIGH to signal start of morse
		delayMicroseconds(40);
		for (uint8_t i = 0; i < pulseCount; i++) {
			digitalWrite(pin, LOW);																						// Each pulse consists of a LOW and a HIGH
			delayMicroseconds(1);																						// They where each 1 microsecond long - in theory...
			digitalWrite(pin, HIGH);																					// In reality the duration varies, depending on microcontroller and other effects
			delayMicroseconds(1);																						// 5 microsecond and single delays up to 13 microseconds are ok
		}
		digitalWrite(pin, HIGH);																						// End with a long HIHG to signal end of morse
		delayMicroseconds(40);
		digitalWrite(pin, LOW);
	#endif
}



// DebugPrintHex -> Print a formatted hex-number (0x01 instead of 0x1)
void gnFellerSwitch::_debugPrintHex(uint8_t value) {
	#ifdef GNFS_DEBUG_SERIAL
		GNFS_DEBUG_SERIAL_PORT.print(F("0x"));
		if (value < 0x10) {
			GNFS_DEBUG_SERIAL_PORT.print(0);
		}
		GNFS_DEBUG_SERIAL_PORT.print(value, HEX);
	#endif
}



// LogError - Sets the LastError Variable and increases errorCount
void gnFellerSwitch::_logError(gnFellerSwitchError error) {
	#ifdef GNFS_DEBUG_DIGITAL
		digitalWrite(GNFS_DEBUG_DIGITAL_PIN_ERR, HIGH);
	#endif
	_lastError = error;
	if (_errorCount < 0xFF) {																							// Don't let the counter go below its maximum value
		_errorCount++;
	}
	#ifdef GNFS_DEBUG_DIGITAL
		digitalWrite(GNFS_DEBUG_DIGITAL_PIN_ERR, LOW);
	#endif
	_debugMorse(GNFS_DEBUG_DIGITAL_PIN_ERR, error);
	#ifdef GNFS_DEBUG_SERIAL
		GNFS_DEBUG_SERIAL_PORT.print(F("ERROR: "));
		_debugPrintHex(error);
		GNFS_DEBUG_SERIAL_PORT.println();
	#endif
}



// LogSwitchControllerError - Sets the LastSwitchControllerError Variable and increases errorCount
void gnFellerSwitch::_logSwitchControllerError(uint8_t error) {
	#ifdef GNFS_DEBUG_DIGITAL
		digitalWrite(GNFS_DEBUG_DIGITAL_PIN_ERR, HIGH);
	#endif
	_lastErrorSwitchController = error;
	if (_errorCountSwitchController < 0xFF) {																			// Don't let the counter go below its maximum value
		_errorCountSwitchController++;
	}
	_debugMorse(GNFS_DEBUG_DIGITAL_PIN_DET, error);
	#ifdef GNFS_DEBUG_DIGITAL
		digitalWrite(GNFS_DEBUG_DIGITAL_PIN_ERR, LOW);
	#endif
}