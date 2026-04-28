#include <Wire.h>
#include <MFRC522_I2C.h>

#define RFID_ADDR 0x28
MFRC522_I2C rfid(RFID_ADDR, -1);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  rfid.PCD_Init();
}

void loop() {
  char uid[32];

  if (Read_RFID(uid, sizeof(uid))) {
    Serial.println(uid);
  }
}

bool Read_RFID(char *buffer, size_t bufferSize) {
  if (!rfid.PICC_IsNewCardPresent()) {
    return false;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    return false;
  }

  size_t index = 0;

  for (byte i = 0; i < rfid.uid.size; i++) {
    // Leading zero if needed
    if (rfid.uid.uidByte[i] < 0x10) {
      if (index < bufferSize - 1) buffer[index++] = '0';
    }

    // Write HEX value
    int written = snprintf(&buffer[index], bufferSize - index, "%X", rfid.uid.uidByte[i]);
    if (written <= 0) break;
    index += written;

    // Add space between bytes
    if (i < rfid.uid.size - 1) {
      if (index < bufferSize - 1) buffer[index++] = ' ';
    }

    // Prevent overflow
    if (index >= bufferSize - 1) break;
  }

  buffer[index] = '\0';  // null terminate
  return true;
}