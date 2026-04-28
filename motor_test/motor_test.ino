#include <Motoron.h>

MotoronI2C mc(16);

// encoder vars
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;

const int encoder1APin = 22;    // left encoder A
const int encoder1BPin = 23;    // left encoder B
const int encoder2APin = 24;    // right encoder A
const int encoder2BPin = 25;    // right encoder B


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Wire.begin();
  
  mc.reinitialize();   
  mc.disableCrc();     
  mc.clearResetFlag(); 
  mc.disableCommandTimeout();
  mc.setMaxAcceleration(1, 200);
  mc.setMaxDeceleration(1, 300);
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 300);

  // encoder interrupts
  pinMode(encoder1APin, INPUT_PULLUP);
  pinMode(encoder1BPin, INPUT_PULLUP);
  pinMode(encoder2APin, INPUT_PULLUP);
  pinMode(encoder2BPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1APin), countEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1BPin), countEncoder2, RISING); 
  attachInterrupt(digitalPinToInterrupt(encoder2APin), countEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2BPin), countEncoder2, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
   // 1. Spin motors FORWARD at maximum speed
  mc.setSpeed(1, 800); // Shield 1, Motor 1
  mc.setSpeed(2, 800); // Shield 1, Motor 2
  delay(2000); 

  // 2. Stop motors
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  delay(1000); 

  Serial.print(encoderCount1);
  Serial.print(" ");
  Serial.println(encoderCount2);

  // 3. Spin motors BACKWARD at maximum speed
  mc.setSpeed(1, -800); // Shield 1, Motor 1
  mc.setSpeed(2, -800); // Shield 1, Motor 2
  delay(2000); 

  // 4. Stop motors
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  delay(1000);

  Serial.print(encoderCount1);
  Serial.print(" ");
  Serial.println(encoderCount2);
}

void countEncoder1() { encoderCount1++; }
void countEncoder2() { encoderCount2++; }


