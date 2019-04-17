/**
 * This Arduino sketch takes a character input from Serial, shifts it by one
 * letter down (A -> B, B -> C, ..., Z -> A), and retransmits over Serial.
 *
 * Only capital letters may be used.
 *
 * No error checking is performed.
 */
 
#define     BAUDRATE    9600

#define     CHAR_BANG   0x21
#define     CHAR_A      0x41
#define     CHAR_Z      0x5A
#define     CHAR_Zp     0x5B
#define     CHAR_a      0x61
#define     CHAR_z      0x7A
#define     CHAR_zp     0x7B
 
char shift (char c);
bool inRange (char c);

char c_rx;

void setup() {
    Serial.begin(BAUDRATE);
}

void loop() {
    if (Serial.available()) {
        c_rx = Serial.read();
        if (inRange(c_rx))
            Serial.print(shift(c_rx));
    }
}

char shift (char c) {
    char c_shift = c + 1;
    if (c_shift == CHAR_Zp)
        c_shift = CHAR_A;
    else if (c_shift == CHAR_zp)
        c_shift = CHAR_a;
    return c_shift;
}

bool inRange (char c) {
    if ( (c >= CHAR_A) && (c <= CHAR_Z) )
        return true;
    if ( (c >= CHAR_a) && (c <= CHAR_z) )
        return true;
    return false;
}
