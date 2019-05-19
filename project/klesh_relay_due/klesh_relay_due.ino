/*
 * This sketch is for a Due serial passthrough between the payload Due and the 
 * xTend radio module. 
 * Serial1 of the relay Due is connected to the payload Due.
 * Serial2 of the relay Due is connected to the payload module 
 * (Due TX to radio pin 5) 
 * (Due RX to radio pin 6)
 */

#define ARDUINO_BAUD 9600
#define RADIO_BAUD 9600

void setup() {
    Serial1.begin(ARDUINO_BAUD);
    Serial2.begin(RADIO_BAUD);
}

void loop() {
    // if hear anything from computer, send it to LKM
    while (Serial1.available() > 0) {
        Serial2.print(Serial1.read());
    }
    // if hear anything from LKM, send it to computer
    while (Serial2.available() > 0) {
        Serial1.print(Serial2.read());
    }
}
