/**
 * This Arduino sketch takes a character input from Serial, shifts it by one
 * letter down (A -> B, B -> C, ..., Z -> A), and retransmits over Serial.
 *
 * Only capital letters may be used.
 *
 * No error checking is performed.
 */
 
#define     BAUDRATE    9600

#define     CHAR_A      0x41
#define     CHAR_Z      0x5A
 
char shift (char c);

char c_rx;

void setup() {
    Serial.begin(BAUDRATE);
}

void loop() {
    if (Serial.available()) {
        char c_rx = (char) Serial.read();
        Serial.print(shift(c_rx));
    }
}

char shift (char c) {
    char c_shift = c + 1;
    if (c_shift > CHAR_Z)
        c_shift = CHAR_A;
    return c_shift;
}
