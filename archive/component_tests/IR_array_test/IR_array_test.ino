const int SENSOR_PIN = 30; 
const int EMITTER_PIN = 40;

void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("--- Giga RC Sensor Manual Test ---");
  
  pinMode(EMITTER_PIN, OUTPUT);
  digitalWrite(EMITTER_PIN, HIGH);
  Serial.println("Emitter turned ON.");
}

void loop() {
  Serial.println("\nStep 1: Setting pin to OUTPUT and driving HIGH...");
  pinMode(SENSOR_PIN, OUTPUT);
  digitalWrite(SENSOR_PIN, HIGH);
  
  // Give the capacitor 10 microseconds to charge
  delayMicroseconds(10);

  Serial.println("Step 2: Switching pin to INPUT...");
  pinMode(SENSOR_PIN, INPUT);

  Serial.println("Step 3: Entering timing loop...");
  unsigned long startTime = micros();
  unsigned long timeOut = startTime + 2500;
  unsigned long elapsed = 0;

  // Wait for the pin to go LOW or for the timeout to hit
  while (micros() < timeOut) {
    if (digitalRead(SENSOR_PIN) == LOW) {
      elapsed = micros() - startTime;
      break;
    }
  }

  Serial.println("Step 4: Loop finished!");
  
  if (elapsed == 0) {
    Serial.println("Result: Timeout (Took >2500us)");
  } else {
    Serial.print("Result: ");
    Serial.print(elapsed);
    Serial.println(" us");
  }

  delay(1000);
}