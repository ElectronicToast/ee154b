
#include <SoftwareSerial.h>

#define USB_BAUD 9600
#define LKM_BAUD 9600

#define SERIAL1_RX 3
#define SERIAL1_TX 4


#define SHUTDOWN_PIN 2

int counter = 0;
long timer = 0;

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
    timer++;
    if (timer == 40000) {
        timer = 0;
        LKMserial.println(counter++);
        if(counter == 1000) {
            counter = 0;
        }
    }
    while(Serial.available()) {
        LKMserial.print((char)Serial.read());
    }
    while(LKMserial.available()) {
        Serial.print((char)LKMserial.read());
    }
}
