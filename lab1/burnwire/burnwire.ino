#define BURN_WIRE_PIN 3

bool burn_wire_on = false;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  pinMode(BURN_WIRE_PIN, OUTPUT);
}

void loop() {
  while (Serial.available() == 0) {}
  while (Serial.available() > 0) {
    Serial.read();
    delay(10);
  }
  burn_wire_on = !burn_wire_on;
  digitalWrite(BURN_WIRE_PIN, burn_wire_on);
}
