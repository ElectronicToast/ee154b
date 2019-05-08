void setup() {
  // put your setup code here, to run once:
  for (int pin = 9; pin < 14; pin++) {
    pinMode(pin, OUTPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int pin = 9; pin < 14; pin++) {
    digitalWrite(pin, HIGH);
    delay(100);  
    digitalWrite(pin, LOW);
  }
}
