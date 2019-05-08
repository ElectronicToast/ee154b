
#include <SoftwareSerial.h>

#define USB_BAUD 9600
#define LKM_BAUD 9600

#define SERIAL1_RX 3
#define SERIAL1_TX 4


#define SHUTDOWN_PIN 2

int counter = 0;

SoftwareSerial LKMserial =  SoftwareSerial(SERIAL1_RX, SERIAL1_TX);

void setup() {
    pinMode(SERIAL1_RX, INPUT);
    pinMode(SERIAL1_TX, OUTPUT);
    pinMode(SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SHUTDOWN_PIN, HIGH);
    Serial.begin(USB_BAUD);
    LKMserial.begin(LKM_BAUD);
}

void loop() {
    while(Serial.available()) {
        LKMserial.write(Serial.read());
    }
    while(LKMserial.available()) {
        Serial.write(LKMserial.read());
    }
}
