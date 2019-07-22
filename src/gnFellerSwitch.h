/*  GN FellerSwitch Library
 *  =======================
 *  
 *  Library for Feller Switches "Feller EDIZIOdue Elektroniktaster (900-3928.FMI.L.61)".
 *  See Readme for details.
 *  
 *  Tested with Arduino UNO and gnFellerTaster PCB (compatible with Arduino Uno).
 *  
 *  2018-09-08  V1.1.2    Andreas Gloor            Reset ErrorCount after Init
 *  2018-07-21  V1.1.1    Andreas Gloor            Initial Version
 *  
 *  Feller and EDIZIOdue are Trademarks of Feller AG, http://www.feller.ch
 *  
 *  MIT License
 *  
 *  Copyright (c) 2018 Andreas Gloor
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
 
 
// Assure the Library gets loaded not more than once.
#ifndef gnFellerSwitch_h
#define gnFellerSwitch_h



// Include necessary Library's
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <Stream.h>
#include <Arduino.h>



// Definitions
#define GNFS_SVC_SetLedState_Variant1							0x11
#define GNFS_SVC_SetLedState_Variant2							0x12
#define GNFS_SVC_SetLedBrightness_Variant1				0x11
			
#define GNFS_TX_BUFFERSIZE												7
#define GNFS_RX_BUFFERSIZE												7

#define GNFS_FLAG_Initialized											0
#define GNFS_FLAG_RequestUpdateForLedState				1
#define GNFS_FLAG_RequestUpdateForLedBrightness		2
#define GNFS_FLAG_ButtonHandler_Raw_Enabled				3
#define GNFS_FLAG_ButtonHandler_Single_Enabled		4



/* Debugging
	 You can enable up to three Debug-Modes.
		* Serial
			 - Is most useful, but dangerous.
			 - It uses a Arduino-Serial interface, that YOU have to initialize in your Sketch (Serial.begin()).
			 - Positive: Much Informations, easy to understand.
			 - Negative/Risk: The use of Serial affects the timing of the code. This maybe even bring the code in an UNFUNCTIONAL state!
		* Digital
			 - Useful with a LogicAnalyzer or maybe a Oscilloscope.
			 - Digital dont consume as much time as Serial. So you can use it with a much lower Risk.
			 - They are 3 Pins which you can define below. Dont use them in your code.
			 - Err goes high if a error occurred. Use getLastError or Morse to get the ErrorCode.
			 - Act signal some activity's. You have to see in the code to understand its "signs".
			 - Det is reserved for some more details.
		* Morse
			 - Morse tells you some codes (Numbers) by pusing them out on a digital pin.
			 - You have to enable Digital to use Morse!
			 - Morse needs also its time, and so it is also dangerous like Serial.
*/
//#define GNFS_DEBUG_SERIAL
//#define GNFS_DEBUG_DIGITAL
//#define GNFS_DEBUG_DIGITAL_ERRORMORSE
#define GNFS_DEBUG_SERIAL_PORT									Serial
#define GNFS_DEBUG_DIGITAL_PIN_ERR							5
#define GNFS_DEBUG_DIGITAL_PIN_ACT							6
#define GNFS_DEBUG_DIGITAL_PIN_DET							7



// Main Class
class gnFellerSwitch {
	public:
		// Constructor and Begin ---------------------------------------------------------------------------------------------
		gnFellerSwitch(HardwareSerial& device);
		gnFellerSwitch(SoftwareSerial& device);
		bool begin() {return begin(57600, 9600);};
		bool begin(uint32_t baudRate) {return begin(baudRate, 9600);};
		bool begin(uint32_t baudRate, uint32_t initialBaudRate);
		
		// Communication Handling --------------------------------------------------------------------------------------------
		void handleCommunication();
		
		// Services ----------------------------------------------------------------------------------------------------------
		enum buttonPosition : uint8_t {
			T1 																															= 1,
			T2 																															= 2,
			T3 																															= 3,
			T4 																															= 4,
			T5 																															= 5,
			T6 																															= 6,
			T7 																															= 7,
			T8 																															= 8};
		enum ledColor : uint8_t {
			red 																														= 0b001001,
			green 																													= 0b010010,
			blue 																														= 0b100100
		};
		enum ledPosition : uint8_t {
			L1 																															= 0,
			L2 																															= 1,
			L3 																															= 2,
			L4 																															= 3,
			L5 																															= 4,
			L6 																															= 5,
			L7 																															= 6,
			L8 																															= 7};
		enum ledState : uint8_t {
			off 																														= 0b000000,
			on 																															= 0b000111,
			blinking 																												= 0b111000
		};
		typedef void (*rawButtonHandlerCallback) (uint8_t rawButtonState);
		typedef void (*singleButtonHandlerCallback) (uint8_t buttonNumber, bool buttonState);
		bool attachButtonHandler(rawButtonHandlerCallback rawButtonHandler);
		bool attachButtonHandler(singleButtonHandlerCallback singleButtonHandler);
		uint8_t getButtonState();
		bool getButtonState(buttonPosition buttonNumber);
		ledColor getLedColor(ledPosition ledNumber);
		ledState getLedState(ledPosition ledNumber);
		void setLed(ledPosition pos, ledState state) {setLed(pos, state, red);};
		void setLed(ledPosition pos, ledState state, ledColor color);
		void setLedBrightness(uint8_t brightness);
		

		
		// Diagnostics -------------------------------------------------------------------------------------------------------
		uint8_t getSwitchFirmwareVersion() {return _switchFirmwareVersion;};
		uint8_t getSwitchLedCount() {return _switchLedCount;};
		uint8_t getSwitchPushButtonCount() {return _switchPushButtonCount;};
		
		// Error Handling ----------------------------------------------------------------------------------------------------
		enum gnFellerSwitchError : uint8_t {
			errNone																													= 0x00,
			errInitInvalidBaudrate																					= 0x10,
			errInitResetFellerSwitchFailed																	= 0x11,
			errInitSetSystemSetingsRequestFailed														= 0x12,
			errInitGetSystemInfoFailed																			= 0x13,
			errInitSwitchFirmwareToLow																			= 0x14,
			errInitSwitchHardwareUnknown																		= 0x15,
			errNotInitializedOrInitFailed																		= 0x20,
			errAnswerFromFellerSwitchInvalid																= 0x21,
			errTimeoutWaitingForFrameFromFellerSwitch												= 0x22,
			errTimeoutWaitingForFrameHeaderFromFellerSwitch									= 0x23,
			errTimeoutWaitingForNextByteFromFellerSwitch										= 0x24,
			errUnexpectedOrCorruptedByteReceivedWhileFrameHeaderExpected		= 0x25,
			errUnexpectedIndicationFrameReceived														= 0x30
		};
		uint8_t getErrorCounter();
		gnFellerSwitchError getLastError();
		uint8_t	getLastSwitchControllerError();
		uint8_t getSwitchControllerErrorCounter();

		
				
	private:
		// Constructor and Begin ---------------------------------------------------------------------------------------------
		uint32_t										_baudRate;
		HardwareSerial*							_hwStream;
    Stream*											_stream;
		SoftwareSerial*							_swStream;
		uint8_t _getFellerBaudrateFlag(uint32_t baudRate);
		
		
		// Communication Handling --------------------------------------------------------------------------------------------
		enum _fellerService : uint8_t {
			noServiceOrReset																								= 0x00,
			setSystemSettings_request																				= 0x10,
			setSystemSettings_confirm																				= 0x11,
			getSystemSettings_request																				= 0x12,
			getSystemSettings_confirm																				= 0x13,
			getSystemState_request																					= 0x18,
			getSystemState_confirm																					= 0x19,
			systemState_indication																					= 0x1A,
			getSystemInfo_request																						= 0x1C,
			getSystemInfo_confirm																						= 0x1D,
			setLedState_request																							= 0x30,
			setLedState_confirm																							= 0x31,
			setLedState_Variant1																						= 0x11,
			setLedState_Variant2																						= 0x12,
			getLedState_request																							= 0x32,
			getLedState_confirm																							= 0x33,
			setLedBrightness_request																				= 0x38,
			setLedBrightness_Variant1																				= 0x11,
			setLedBrightness_confirm																				= 0x39,
			getLedBrightness_request																				= 0x3A,
			getLedBrightness_confirm																				= 0x3B,
			getButtonState_request																					= 0x40,
			getButtonState_confirm																					= 0x41,
			buttonState_indication																					= 0x42
		};
		enum _receiveAction : uint8_t {
			receiveAndCallback																							= 0x01,
			demandSpecificServiceRetryOnce																	= 0x02,
			demandSpecificServiceFail																				= 0x04,
			demandReset																											= 0x08
		};
		uint32_t										_ctsTimer = millis();
		uint8_t											_rxBuffer[GNFS_RX_BUFFERSIZE];
		uint8_t											_rxBufferPayloadCount;
		uint8_t											_txBuffer[GNFS_TX_BUFFERSIZE];
		void _beginCtsPause();
		void _beginSerial(uint32_t baudRate);
		uint8_t _bitCount(uint8_t bit);
		uint8_t _calculateFrameHeader(uint8_t payloadBytesCount, _fellerService service);
		uint16_t _calculateInterByteTimeout() {return ((100000ul / _baudRate) + 1);};
		uint16_t _calculateInterFrameTimeout() {return (_calculateInterByteTimeout() * 2);};	// As told by MS from Feller
		bool _checkCts();
		bool _checkFrameHeaderParity(uint8_t frameHeader);
		void _handleIndicationFrame(uint8_t indicationService);
		bool _receiveFrame(_receiveAction action, _fellerService exceptedService = noServiceOrReset);
		bool _sendFrame(_fellerService sendService, uint8_t payloadByteCount, _fellerService responseService = noServiceOrReset);
		bool _tryReset() {return (_sendFrame(noServiceOrReset, 0));};
		
		// Services ----------------------------------------------------------------------------------------------------------
		uint8_t											_brightness = 0xFF;
		uint8_t											_buttonState = 0;
		uint8_t											_ledState[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};	// According to SetLedState Variant2 format
		rawButtonHandlerCallback 		_rawButtonHandler;
		singleButtonHandlerCallback _singleButtonHandler;
		bool _updateLedBrightness();
		bool _updateLedState();
		
		// Diagnostics -------------------------------------------------------------------------------------------------------
		uint8_t _switchFirmwareVersion;
		uint8_t _switchLedCount;
		uint8_t _switchPushButtonCount;
		
		// Error Handling ----------------------------------------------------------------------------------------------------
		uint8_t											_errorCount = 0;
		uint8_t											_errorCountSwitchController = 0;
		gnFellerSwitchError					_lastError = errNone;
		uint8_t											_lastErrorSwitchController = 0;
		void _debugMorse(uint8_t pin, uint8_t pulseCount);
		void _debugPrintHex(uint8_t value);
		void _logError(gnFellerSwitchError error);
		void _logSwitchControllerError(uint8_t error);
		
		// Flag handling -----------------------------------------------------------------------------------------------------
		uint8_t											_flags = 0x00;	// Initialized, RequestUpdateForLedState, RequestUpdateForBrightness
		void _clearFlag(uint8_t flag) {_writeFlag(flag, false);};
		bool _readFlag(uint8_t flag) {return bitRead(_flags, flag);};
		void _setFlag(uint8_t flag) {_writeFlag(flag, true);};
		void _writeFlag(uint8_t flag, bool value) {bitWrite(_flags, flag, value);};
};
#endif	// #ifndef gnFellerSwitch_h