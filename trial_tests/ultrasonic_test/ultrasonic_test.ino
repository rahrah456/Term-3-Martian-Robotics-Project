const int TRIG = 11;
const int ECHO = 12;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.println("Ultrasonic test — distance in cm");
}

void loop() {
  long d = measureCM();
  Serial.print("Distance: ");
  Serial.print(d);
  Serial.println(" cm");
  delay(200);
}

long measureCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long us = pulseIn(ECHO, HIGH, 30000);
  if (us == 0) return 999;
  return us / 58;
}
