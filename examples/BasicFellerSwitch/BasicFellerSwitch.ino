/*  GN FellerSwitch Library
 *  =======================
 *  
 *  Library for Feller Switches "Feller EDIZIOdue Elektroniktaster (900-3928.FMI.L.61)".
 *  
 *  The BasicFellerSwitch example demonstrates the possibility's of the Library and the Feller Switch.
 *  See Readme for details.
 *  
 *  Tested with Arduino UNO.
 *  Baudrate: 115200baud to PC (9600baud to Switch)
 *  
 *  Arduino Uno     Feller EDIZIOdue Elektroniktaster
 *  5V              5V
 *  GND             GND
 *  A0              RxD
 *  A1              TxD
 *  A4              Potentiometer between VCC and GND to control L7 LED
 *  A5              Potentiometer between VCC and GND to control Brightness (connect to VCC if you dont have a pot)
 *
 *  
 *  2018-07-21  V1.0.1    Andreas Gloor            Initial Version
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
 
 
#define     TX_PIN                  A0                                                                                // Define Pin's for Communication with the Feller-Switch.
#define     RX_PIN                  A1

#define     POTBRIGHTNESS_PIN       A5                                                                                // Attach a Potentiometer to this Pin to control the brightness.
#define     POTL7COLOR_PIN          A4                                                                                // Attach a Potentiometer to this Pin to control the color of L7.
                                                                                                                      // Notice: this pins are not neccessary.

#include <SoftwareSerial.h>                                                                                           // Used for Communication with the Feller-Switch.
SoftwareSerial serialFeller(RX_PIN, TX_PIN);

#include "gnFellerSwitch.h"                                                                                           // FellerSwitch Library
gnFellerSwitch fellerSwitch(serialFeller);

uint32_t lastLedUpdateTimer = 0;                                                                                      // Delay-like timer, avoid blocking



void setup() {
  Serial.begin(115200);                                                                                               // Setting up Serial to PC with different Baud-Rate and wait till ready.
  while (!Serial) {}

  pinMode(POTBRIGHTNESS_PIN, INPUT);                                                                                  // Initializing the Potentiometer-Pin.
  pinMode(POTL7COLOR_PIN, INPUT);

  Serial.println(F("BasicFellerSwitch"));                                                                             // Printing out the "Header".
  Serial.println(F("*****************"));
  Serial.println();

  Serial.print(F("Initializing FellerSwitch..."));
  fellerSwitch.attachButtonHandler(handleRaw);                                                                        // Attach both type of Button-Change-Event Handlers. See the explanations below for details.
  fellerSwitch.attachButtonHandler(handleSingle);                                                                     // Notice: its most likely nonsens to attach both type of Handlers. We do this here just for demo.
  if (fellerSwitch.begin(115200)) {                                                                                   // BaudRate is optional. Default is 9600.
                                                                                                                      // If you changed the Default Baudrate by adding a resistor on the FellerSwitch, you have to specify the new Value here as a second argument.
    Serial.println(F("\t[OK]"));

    Serial.print(F("This Switch has "));                                                                              // Like this you can read out the capability's of the FellerSwitch.
    Serial.print(fellerSwitch.getSwitchPushButtonCount());
    Serial.print(F(" Push-Buttons and "));
    Serial.print(fellerSwitch.getSwitchLedCount());
    Serial.println(F(" LED's."));
    Serial.print(F("Its Firmware Version is V"));
    Serial.print(fellerSwitch.getSwitchFirmwareVersion() >> 4);                                                       // Firmware-Version is decoded as <4 Major-Bits><4 Minor-Bits>.
    Serial.print(F("."));
    Serial.print(fellerSwitch.getSwitchFirmwareVersion() & 0x0F);
    Serial.print(F("."));
    Serial.println();
    Serial.println();
  } else {
    Serial.println(F("\t[FALED]"));
    Serial.print(F("### ERR: 0x")); Serial.println(fellerSwitch.getLastError(), HEX);
    delay(100000);
  }
}

void loop() {
  
  fellerSwitch.handleCommunication();                                                                                 // Update the Feller-Switch. Keep in mind, that communication occures only if neccessary (something realy changed!).

  gnFellerSwitch::gnFellerSwitchError lastError = fellerSwitch.getLastError();                                        // You can always check for errors and use them.
  if (lastError != gnFellerSwitch::errNone) {                                                                         // Keep care that a query of getLastError() will reset the error.
    Serial.print(F("### ERR: 0x")); Serial.println(lastError, HEX);                                                   // Its everytime the LAST error, so be aware of the context you use it.
    Serial.print(F("This was one of ")); Serial.print(fellerSwitch.getErrorCounter()); Serial.println(F(" errors since last query."));
  }
  uint8_t lastSwitchControllerError = fellerSwitch.getLastSwitchControllerError();                                    // Its also possible to read out occurred errors from the FellerSwitch
  if (lastSwitchControllerError != 0x00) {
    Serial.print(F("### SwitchControllerErr: 0x")); Serial.println(lastSwitchControllerError, HEX);
    Serial.print(F("This was one of ")); Serial.print(fellerSwitch.getSwitchControllerErrorCounter()); Serial.println(F(" errors since last query."));
  }

    
  if (millis() - lastLedUpdateTimer > 2000) {                                                                         // Change something every 2 seconds
    Serial.print(F("RAW Button State: "));                                                                            // Bevore Change: read out some Values to demonstrate these methods.
    printBinaryByte(fellerSwitch.getButtonState());
    Serial.println();
    Serial.print(F("Button State of T7: "));
    Serial.println(fellerSwitch.getButtonState(gnFellerSwitch::T7));
    Serial.print(F("L7 "));
    switch (fellerSwitch.getLedState(gnFellerSwitch::L7)) {
      case gnFellerSwitch::off:
        Serial.print(F("is off"));
        break;
      case gnFellerSwitch::on:
        Serial.print(F("lights"));
        break;
      case gnFellerSwitch::blinking:
        Serial.print(F("blinks"));
        break;
      default:
        Serial.print(F("oups"));
        break;
    }
    if (fellerSwitch.getLedState(gnFellerSwitch::L7) != gnFellerSwitch::off) {
      Serial.print(F(" in "));
      switch (fellerSwitch.getLedColor(gnFellerSwitch::L7)) {
        case gnFellerSwitch::red:
          Serial.print(F("red"));
          break;
        case gnFellerSwitch::green:
          Serial.print(F("green"));
          break;
        case gnFellerSwitch::blue:
          Serial.print(F("blue"));
          break;
        default:
          Serial.print(F("oups"));
          break;
      }
    }
    Serial.println(F("."));
    
    fellerSwitch.setLed(gnFellerSwitch::L1, gnFellerSwitch::on, gnFellerSwitch::red);                                 // Demonstrate the Use of some LED-Functions. Dont forget to invoke the update() Method after chaning!
    fellerSwitch.setLed(gnFellerSwitch::L3, gnFellerSwitch::on, gnFellerSwitch::green);
    fellerSwitch.setLed(gnFellerSwitch::L5, gnFellerSwitch::on, gnFellerSwitch::blue);
    fellerSwitch.setLed(gnFellerSwitch::L2, gnFellerSwitch::blinking, gnFellerSwitch::red);
    fellerSwitch.setLed(gnFellerSwitch::L4, gnFellerSwitch::blinking, gnFellerSwitch::green);
    fellerSwitch.setLed(gnFellerSwitch::L6, gnFellerSwitch::blinking, gnFellerSwitch::blue);
    Serial.println(F("Updated L1, L3, L5 to RGB-on. Updated L2, L4, L6 to RGB-blinking."));
      
    uint8_t newBrightness = map(analogRead(POTBRIGHTNESS_PIN), 0, 1023, 0, 253) & 0b11111100;                         // Set Brightness depending on Potentiometer (drop the two last bytes to avoid flipping)
    fellerSwitch.setLedBrightness(newBrightness);
    Serial.print(F("Set Brightness to ")); Serial.println(newBrightness);
    
    uint8_t newColor = map(analogRead(POTL7COLOR_PIN), 0, 960, 0, 6);                                                 // Set Color and State of L7 depending on Potentiometer
    Serial.print(F("Set L7 to "));
    switch (newColor) {
      case 6:
        fellerSwitch.setLed(gnFellerSwitch::L7, gnFellerSwitch::blinking, gnFellerSwitch::blue);
        Serial.println(F("Blue-Blinking"));
        break;
      case 5:
        fellerSwitch.setLed(gnFellerSwitch::L7, gnFellerSwitch::blinking, gnFellerSwitch::green);
        Serial.println(F("Green-Blinking"));
        break;
      case 4:
        fellerSwitch.setLed(gnFellerSwitch::L7, gnFellerSwitch::blinking, gnFellerSwitch::red);
        Serial.println(F("Red-Blinking"));
        break;
      case 3:
        fellerSwitch.setLed(gnFellerSwitch::L7, gnFellerSwitch::on, gnFellerSwitch::blue);
        Serial.println(F("Blue"));
        break;
      case 2:
        fellerSwitch.setLed(gnFellerSwitch::L7, gnFellerSwitch::on, gnFellerSwitch::green);
        Serial.println(F("Green"));
        break;
      case 1:
        fellerSwitch.setLed(gnFellerSwitch::L7, gnFellerSwitch::on, gnFellerSwitch::red);
        Serial.println(F("Red"));
        break;
      default:
        fellerSwitch.setLed(gnFellerSwitch::L7, gnFellerSwitch::off);
        Serial.println(F("Off"));
        break;
    }

    Serial.println();
    Serial.flush();
    lastLedUpdateTimer = millis();
  }
}



/*  Informations about Button-Change-Handlers
 *  =========================================
 *  
 *  You have two possible ways to take care about button change events:
 *   1. Use a RAW-Handler
 *   2. Use a SINGLE-Handler
 *  
 *  RAW gets one Byte of Data on each Change-Event, containing the current state of each button. So you have to check by yourselv
 *  what has changed and which button it was. This maybe more complicated to you, but may also be faster. 
 *  Example: If you plan to forward a Byte with all Buttons to a MQTT broker, the RAW Method shoud be your choice.
 *  
 *  Single gets a call for every single button and its change. This is more comfortable because you dont have to care about last state.
 *  The disadvantage of this method may be the chatiness when many events occurre. But that will depend mostly on your code.
 *  
 *  Notice:  You always get a press-event and a release-event.
 *  Caution: You have to deal with a possible packet-loss during heavy communications.
 *           This can result in a loss of a complete press/release cycle.
 *           But worst case will be that you only get the pressd-event and the release-event triggers next time a (other) button pressed.
 *  Notice:  The buttons are debounced allready by the switch.
 *  Notice:  You can attach both type of handlers at the same. But you cannot attach two times the same type of handler.
 *           In this case only the last attached will be called.
 *  Notice:  Buttons labled T1..T8. The Parameters/Values for a single Button are 1..8, corresponding to the T-Number.
 *           See Feller Documentation about which button has wich T-Number.
 */
void handleRaw(uint8_t rawButtonState) {                                                                              // Ths is a callback-function which will be calles from FellerSwitch on a Button-Change-Event
  Serial.print(F("Buttons changed. In a raw-handler you have to care yourself about what changed. MSB=T8, LSB=T1. This is the actual State: "));
  printBinaryByte(rawButtonState);
  Serial.println();
}



void handleSingle(uint8_t buttonNumber, bool buttonState) {                                                           // Ths is a callback-function which will be calles from FellerSwitch on a Button-Change-Event
  Serial.print(F("Single Button changed. In a single-handler you have more informations, but get possible more calls. Button T"));
  Serial.print(buttonNumber);
  Serial.print(F(" has changed to "));
  Serial.print(buttonState);
  Serial.println(F("."));
}



void printBinaryByte(uint8_t byteToPrint) {                                                                            // This is just a Debug-Function which print all bits of a Byte
  for (int8_t i = 7; i >= 0; i--) {
    if (i == 3) {
      Serial.print(F(" "));
    }
    Serial.print(bitRead(byteToPrint, i));
  }
}

