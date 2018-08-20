/*  UnoSerialHexDebug
 *  =================
 *  
 *  Tool to manually test the Feller Interface, working wit HEX-Codes.
 *  
 *  Baudrate: 115200baud to PC (9600baud to Switch)
 *  
 *  The Switch has to use with open COMTYPE/BAUDRATE Pin (= Default-Values).
 *  
 *  Arduino Uno     Feller EDIZIOdue Elektroniktaster
 *  5V              5V
 *  GND             GND
 *  A0              RxD
 *  A1              TxD
 *  
 *  2018-06-10  Andreas Gloor                       Initial Version
 *  Licence: CC-BY
 */


#include <SoftwareSerial.h>                                                                                           // Used for Communication with the Feller-Switch.
SoftwareSerial feller(A1, A0); // RX, TX

byte            initByte;                                                                                             // Declaring and initializing Variables.
byte            hByte;
byte            lByte;
byte            readedByte;
unsigned long   timeStamp                     = 0;


void setup() {
  Serial.begin(115200);                                                                                               // Setting up the Serials with different Baud-Rates and wait for USB-Serial.
  feller.begin(9600);
  while (!Serial) {
  }

  Serial.println(F("UnoSerialHexDebug"));                                                                             // Printing out the "Manual".
  Serial.println(F("Enter Commands in Format ""0x21 0x12""."));
  Serial.println();
}


void loop() {
  if( feller.available()) {                                                                                           // Getting some Data from the Feller-Switch...
    readedByte = feller.read();                                                                                       // Reat the Byte.
    Serial.print(F("< \t0x"));                                                                                        // Write them to the PC as HEX and DEC Value.
    print2digitHex(readedByte);
    Serial.print(F("\t\t"));
    print2x4digitBin(readedByte);
    Serial.print(F("\t"));
    printTimeStamp();
    Serial.println();
  }
  

  if (Serial.available()) {                                                                                           // Getting some Data from PC...
    initByte = Serial.read();                                                                                         // Read first Byte.

    if (initByte == 0x78) {                                                                                           // Triggering for the x (in 0x21) for Example.
      Serial.print(F("> 0x"));                                                                                        // Signalize the received Byte begin.
    
      while (Serial.available() < 2) {                                                                                // Wait till both ASCII-Characters of the Byte arrived.
        delay(10);
      }
    
      hByte = asciiHexToDec(Serial.read());                                                                           // Read both ASCII-Characters of the Byte, immeadiatly convert them into a 4bit Part.
      lByte = asciiHexToDec(Serial.read());
      if (hByte == 0xFF || lByte == 0xFF) {                                                                           // Invalid Character received. Display Error an skip this Byte.
        Serial.println(F("ERR"));
        return;
      }
      hByte = hByte << 4;                                                                                             // Shifting the higher 4bit-Part to the High and Bitwise-OR both 4bit-Parts together.
      readedByte = hByte | lByte;

      feller.write(readedByte);                                                                                       // Write-Out the Byte to the Feller-Switch.
      timeStamp = millis();                                                                                           // Reset the TimeStamp.
      
      print2digitHex(readedByte);                                                                                     // Confirm (Echo) the reading.
      Serial.print(F("\t\t\t"));
      print2x4digitBin(readedByte);
      Serial.print(F("\t"));
      printTimeStamp();
      Serial.println();
    }
  }
}


byte asciiHexToDec(byte input) {                                                                                    // Convert a ASCII Char from HEX to DEC.
  if (input >= 0x30 && input <= 0x39) {                                                                               // 0-9
    return input - 0x30;
  }
  else if (input >= 0x41 && input <= 0x46) {                                                                          // A-F
    return input - 0x41 + 10;
  }
  else if (input >= 0x61 && input <= 0x66) {                                                                          // a-f
    return input - 0x61 + 10;
  }
  else {                                                                                                              // Invalid
    return 0xFF;
  }
}


void print2digitHex(byte input) {                                                                                   // Prints a Byte out to Serial, using 2 Hex-Chars.
  if (input < 0xF) {                                                                                                  // If Value < 0xF, print leading Zero.
    Serial.print(0);
  }
  Serial.print(input, HEX);
}


void print2x4digitBin(byte input) {                                                                                 // Prints a Byte out to Serial, using 2 Hex-Chars.
  for (byte i=7; i!=0xFF; i--) {                                                                                      // Iterate Bitwise.
    if (i == 3) {                                                                                                     // Insert a Space after 4 Bits.
      Serial.print(F(" "));
    }
    Serial.print(input >> i & 1);                                                                                     // Shift the relevant Bit to the lowes Position and AND them with 1 to print its Value.,
  }
}


void printTimeStamp() {                                                                                             // Prints the TimeStamp in ###0.000s Format
  unsigned long currentTime;                                                                                          // Get TimeStamp Value.
  currentTime = millis() - timeStamp;
  unsigned long currentPart;                                                                                          // Get the first Part (Seconds).
  currentPart = currentTime / 1000;
  
  if (currentPart < 100) {                                                                                            // If neccessary, write-out leading Spaces.
    Serial.print(F(" "));
  }
  if (currentPart < 10) {
    Serial.print(F(" "));
  }
  Serial.print(currentPart);                                                                                          // Write-Out the Value.
  Serial.print(F("."));                                                                                               // Write-Out the decimal point.
  
  currentPart = currentTime % 1000;                                                                                   // Get the second Part (Milliseconds).
  if (currentPart < 100) {                                                                                            // If neccessary, write-out leading Zeros.
    Serial.print(F("0"));
  }
  if (currentPart < 10) {
    Serial.print(F("0"));
  }
  Serial.print(currentPart);                                                                                          // Write-Out the Value.
  Serial.print(F("s"));
}
