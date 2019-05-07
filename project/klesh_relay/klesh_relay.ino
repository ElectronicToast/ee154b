/*
 * This sketch is meant to relay responses between a host computer and the LKM.
 * The computer connects to an Uno's Serial port, while the LKM connects to
 * a software serial port on pins 2 (RX) and 3 (TX).
 * 
 * Uno Serial:
 *      Pin 0 RX -> USB
 *      Pin 1 TX -> USB
 * Uno software serial
 *      Pin 2 RX -> LKM TX
 *      Pin 3 TX -> LKM RX
 */

#include <SoftwareSerial.h>

#define USB_BAUD 9600
#define LKM_BAUD 9600

#define SERIAL1_RX 2
#define SERIAL1_TX 3

SoftwareSerial LKMserial =  SoftwareSerial(SERIAL1_RX, SERIAL1_TX);

void setup() {
    pinMode(SERIAL1_RX, INPUT);
    pinMode(SERIAL1_TX, OUTPUT);
    Serial.begin(USB_BAUD);
    LKMserial.begin(LKM_BAUD);
}

void loop() {
    // if hear anything from computer, send it to LKM
    while (Serial.available() > 0) {
        LKMserial.print(Serial.read());
    }
    // if hear anything from LKM, send it to computer
    while (LKMserial.available() > 0) {
        Serial.print(LKMserial.read());
    }
}
