#define Hall_Sensor_Pin A0

void setup() {
  Serial.begin(115200);
}

void loop() {
  long total = 0;

  for (int i = 0; i < 100; i++) {
    total += analogRead(Hall_Sensor_Pin);
    delay(1);
  }

  float avg = total / 100.0;

  Serial.print(avg);
  Serial.print(" | ");
  Serial.println(avg > 860.04 ? "Seed detected" : "No seed");

  delay(100);
}