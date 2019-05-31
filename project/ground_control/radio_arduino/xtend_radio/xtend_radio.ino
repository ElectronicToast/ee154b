/* This sketch makes the Arduino act as a UART passthrough from the host
 * computer's USB to the xTend vB RF Module. 
 
 * Radio side:   Arduino Uno side:
 * PIN 1 GND    | GND
 * PIN 2 VCC    | 5V
 * PIN 5 DIN    | DIO PIN 4 (RADIO_TX)
 * PIN 6 DOUT   | DIO PIN 3 (RADIO_RX)
 * PIN 7 SHDN   | DIO PIN 2 (SHUTDOWN_PIN)
 */

#include <SoftwareSerial.h>

#define USB_BAUD 9600
#define RADIO_BAUD 9600

#define RADIO_RX 3
#define RADIO_TX 4

#define SHUTDOWN_PIN 2

SoftwareSerial radioSerial =  SoftwareSerial(RADIO_RX, RADIO_TX);

void setup() {
    pinMode(RADIO_RX, INPUT);
    pinMode(RADIO_TX, OUTPUT);
    pinMode(SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SHUTDOWN_PIN, HIGH);
    Serial.begin(USB_BAUD);
    radioSerial.begin(RADIO_BAUD);
}

void loop() {
    while(Serial.available()) {
        radioSerial.print((char)Serial.read());
    }
    while(radioSerial.available()) {
        Serial.print((char)radioSerial.read());
    }
}
