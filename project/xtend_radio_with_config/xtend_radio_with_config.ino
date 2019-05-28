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

    // begin radio baudrate setup
    configureRadio();
//    radioSerial.print("+++");
//    while (radioSerial.available() == 0) {
//      
//    }
//    while(radioSerial.available()) {
//      Serial.print((char)radioSerial.read());
//      delay(100);
//    }
    
}

void loop() {
    
    while(Serial.available()) {
        char c = (char)Serial.read();
        if (c == 'Q') {
            radioSerial.print("ATCN\r");
        }
        else {
            radioSerial.print(c);
        }
        
    }
    while(radioSerial.available()) {
        Serial.print((char)radioSerial.read());
    }
}

// configure radio for 10 kb/s data rate
bool configureRadio() {
    Serial.println("Begin configuring radio.");
    delay(1000);
    radioSerial.print("ATCN\r");
    String msg = readRadio();
    delay(1000);
    radioSerial.print("+++");
    msg = readRadio();
    if (msg.indexOf("OK") != -1) {
        Serial.println("Entered command mode.");
        delay(1000);
        radioSerial.print("ATBR0\r");
        msg = readRadio();
        if (msg.indexOf("OK") != -1) {
            Serial.println("Configured 10 kb/s data rate");
            delay(1000);
            radioSerial.print("ATCN\r");
            msg = readRadio();
            if (msg.indexOf("OK") != -1) {
                Serial.println("Exited command mode.");
            }
            else {
                Serial.println("Failed to exit command mode");
            }
        }
        else {
            Serial.println("Failed to configure data rate");
        }
    }
    else {
        Serial.println("Failed to enter command mode.");
    }
}

String readRadio() {
    long startTime = millis();
    while (radioSerial.available() == 0 && millis() - startTime < 3000) {}
    String msg = "";
    if (radioSerial.available()) {
        while (radioSerial.available())
        {
            msg += (char)radioSerial.read();
            delay(10);
        }
    }
    else {
        msg = "TO";
    }

    return msg;
}
