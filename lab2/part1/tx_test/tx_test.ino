/**
 * Test sketch - sends A every second
 */
 
#define     BAUDRATE    9600

#define     CHAR_A      0x41
#define     CHAR_Z      0x5A
#define     CHAR_a      0x61
#define     CHAR_z      0x7A
 

void setup() {
    Serial.begin(BAUDRATE);
}

void loop() {
    Serial.print('A');
    delay(1000);
}
