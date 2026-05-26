#include <Wire.h>

void setup() {
  Wire1.begin();
  Serial.begin(115200);
  Serial.println("Scanning...");
}

void loop() {
  for (byte addr = 1; addr < 127; addr++) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      Serial.print("Found: 0x");
      Serial.println(addr, HEX);
    }
  }
  delay(3000);
}