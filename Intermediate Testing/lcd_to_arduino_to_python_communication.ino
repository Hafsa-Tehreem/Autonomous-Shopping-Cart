#include <SoftwareSerial.h>

// Connect DWIN TX -> Pin 6, DWIN RX -> Pin 7
SoftwareSerial dwinSerial(6, 7);

byte frame[9];

void setup() {
  Serial.begin(115200);     // USB Serial to Python
  dwinSerial.begin(115200); // Default DWIN DGUS baud rate (change to 9600 if configured in DGUS)
  
  Serial.println("READY: Waiting for DWIN LCD touch events...");
}

void loop() {
  // Check if at least 9 bytes (standard DWIN button return frame) are available
  if (dwinSerial.available() >= 9) {
    // Look for the 0x5A 0xA5 header
    if (dwinSerial.read() == 0x5A) {
      if (dwinSerial.read() == 0xA5) {
        frame[0] = 0x5A;
        frame[1] = 0xA5;
        
        // Read the remaining 7 bytes of the packet
        for (int i = 2; i < 9; i++) {
          while (!dwinSerial.available());
          frame[i] = dwinSerial.read();
        }

        // Command 0x83 = VP Data Return (touch event from LCD)
        if (frame[3] == 0x83) {
          uint16_t vpAddress = (frame[4] << 8) | frame[5];
          uint16_t keyValue  = (frame[7] << 8) | frame[8];

          // Send formatted message over USB Serial to Python
          Serial.print("DWIN_EVENT|VP:0x");
          Serial.print(vpAddress, HEX);
          Serial.print("|VAL:");
          Serial.println(keyValue);
        }
      }
    }
  }
}
