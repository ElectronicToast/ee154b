/*
 * This sketch is meant to relay responses between a host computer and the LKM.
 * The computer connects to a Due's Serial port 0, while the LKM connects
 * to the Due's Serial port 1.
 * 
 * Due Serial0:
 *      TX -> USB
 *      RX -> USB
 * Due Serial1:
 *      TX -> LKM RX
 *      RX -> LKM TX
 */

#define USB_BAUD 9600
#define LKM_BAUD 9600

void setup() {
    Serial.begin(USB_BAUD);
    Serial1.begin(LKM_BAUD);
}

void loop() {
    // if hear anything from computer, send it to LKM
    while (Serial.available() > 0) {
        Serial1.print(Serial.read());
    }
    // if hear anything from LKM, send it to computer
    while (Serial1.available() > 0) {
        Serial.print(Serial1.read());
    }
}
