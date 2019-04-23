/**
 * This Arduino sketch takes a character input from Serial, shifts it by one
 * letter down (A -> B, B -> C, ..., Z -> A), and retransmits over Serial.
 *
 * Only capital letters may be used. If invalid input is received, a '!' is 
 * retransmitted over Serial.
 */
 
#define     BAUDRATE    9600

#define     CHAR_A      0x41
#define     CHAR_Z      0x5A
#define     CHAR_Zp     0x5B        // One hex value past Z
#define     CHAR_a      0x61
#define     CHAR_z      0x7A
#define     CHAR_zp     0x7B        // One hex value past z
 
char shift (char c);

char c_rx;

void setup() {
    Serial.begin(BAUDRATE);
}

void loop() {
    if (Serial.available()) {
        c_rx = Serial.read();
        if(inRange(c_rx))
            Serial.print(shift(c_rx));      // Send shifted input if valid
        else
            Serial.print(CHAR_BANG);        // or invalid character if not
    }
}

/* This function shifts a character 'c' by one (e.g. a -> b, Z -> A). Valid
 * character input 'c' is assumed.
 */
char shift (char c) {
    char c_shift = c + 1;       // Shift by incrementing
    // Loop over if out of range
    if (c_shift == CHAR_Zp)
        c_shift = CHAR_A;
    else if (c_shift == CHAR_zp)
        c_shift = CHAR_a;
    return c_shift;
}

/* This function returns true if 'c' is a valid character (a-z, A-Z) and 
 * false otherwise.
 */
bool inRange (char c) {
    // Check if uppercase
    if ( (c >= CHAR_A) && (c <= CHAR_Z) )
        return true;
    // Check if lowercase
    if ( (c >= CHAR_a) && (c <= CHAR_z) )
        return true;
    return false;
}