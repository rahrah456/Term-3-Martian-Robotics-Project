#include <Servo.h>

Servo myservo;

const int angles[] = {0, 25, 65, 105, 145, 180};
int angleIdx = 0;

void setup() {
  myservo.attach(31);
  myservo.write(angles[angleIdx]);
  angleIdx += 1;
}

void loop() {
  Drop_Seed();
  delay(10000);
}

void Drop_Seed() {
  myservo.write(angles[angleIdx]);
  angleIdx = (angleIdx + 1) % 6;
}