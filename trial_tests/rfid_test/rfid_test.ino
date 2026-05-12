#include <Wire.h>
#include <MFRC522_I2C.h>

#define RFID_ADDR 0x28
MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);

void setup() {
  Serial.begin(9600);
  Wire1.begin();
  rfid.PCD_Init();
  Serial.println("RFID test — tap a tag");
}

void loop() {
  char uid[32];

  if (readTag(uid, sizeof(uid))) {
    Serial.print("Tag: ");
    Serial.println(uid);
  }

  delay(100);
}

bool readTag(char *buf, size_t len) {
  if (!rfid.PICC_IsNewCardPresent()) return false;
  if (!rfid.PICC_ReadCardSerial()) return false;

  size_t idx = 0;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) {
      if (idx < len - 1) buf[idx++] = '0';
    }
    int n = snprintf(&buf[idx], len - idx, "%X", rfid.uid.uidByte[i]);
    if (n <= 0) break;
    idx += n;
    if (i < rfid.uid.size - 1) {
      if (idx < len - 1) buf[idx++] = ' ';
    }
    if (idx >= len - 1) break;
  }
  buf[idx] = '\0';
  return true;
}
