const int killSwitch = 38;
const int activation = 39;

void setup() {
  // put your setup code here, to run once:
  pinMode(killSwitch, INPUT_PULLUP);
  pinMode(activation, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Check_Death();
}

void Check_Death(){
  if(digitalRead(killSwitch)==HIGH){
    while(digitalRead(killSwitch)==HIGH){
      delay(50);
    }
    Death(); 
  }
}

void Death(){
  digitalWrite(activation, LOW);
  while(digitalRead(killSwitch)==LOW){
    delay(50);
  }

  while(digitalRead(killSwitch)==HIGH){
    delay(50);
  }
  digitalWrite(activation, HIGH);
}
