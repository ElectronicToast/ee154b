
#define SHUTDOWN_PIN 2


void setup() {
    pinMode(SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SHUTDOWN_PIN, HIGH);
    Serial.begin(9600);
}

void loop() {
    while(Serial.available() > 0) {
        Serial.print(Serial.read());
    }
}