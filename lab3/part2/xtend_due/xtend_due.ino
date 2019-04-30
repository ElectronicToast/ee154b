/*
 * xtend_due.ino - Sketch for interfacing a XTEND module with a laptop serial 
 * client using UART
 *
 * The XTEND module is connected to the hardware Serial1 on the Arduino Due,
 * while Serial (the USB Serial) is used to connect the Due to the client 
 * computer.
 *
 * Serial TX: Computer USB
 * Serial RX: Computer USB
 *
 * Serial1 TX: XTEND RS232 (with RS232 breakout cable to plug into Due)
 * Serial1 RX: XTEND RS232
 *
 * The RSSI value is read and computed whenever any input from the laptop 
 * client is received
 */

int counter;
int errors;

#define     XTD_BAUD    2400            // Configure to match XTEND baudrate
#define     USB_BAUD    9600            // Configure to match client baudrate

#define     RSSI_PIN        7
#define     RSSI_NREADS     8           // # of reads to average for RSSI 
#define     RSSI_PERIOD_US  8320        // PWM period in us, p.47 of datasheet
#define     RSSI_TIMEOUT_US 20000   

#define     RSSI_START      '#'         // Delimiter serial characters
#define     RSSI_END        '#'

#define     STARTUP_DEL     2000


float computePWM(int pin, int nTrials){
    // Averages high-pulse lengths over RSSI_NREADS reads
    float pulseSum = 0.0;
    for(int i = 0; i < nTrials - 1; i++){
        pulseSum += pulseIn(pin, HIGH, RSSI_PERIOD_US);
    }
    return pulseSum / nTrials / RSSI_PERIOD_US;
}


void setup()
{
    // Initialize communication with computer and radio
    Serial.begin(USB_BAUD);
    Serial1.begin(XTD_BAUD);
    
    // Wait a bit 
    delay(STARTUP_DEL);
}


void loop()
{ 
    // Read client command 
    if (Serial.available() > 0) {
        // Read the RSSI pin pulse high time 
        float rssi_frac =  computePWM(RSSI_PIN, RSSI_NREADS);
        
        /* Rough gain scale
         * 10 dB (30%), 20 dB (45%), 30 dB (60%)
         */
        int rssi_db = rssi_frac * (200/3) - 10;
        
        // Send the RSSI value
        Serial.print(RSSI_START);
        Serial.print(rssi_db);
        Serial.print(RSSI_END);
        
        // Relay to XTEND
        while (Serial.available() > 0) {
            Serial1.print(Serial.read());
        }            
    }
    
    // Read back received data 
    if (Serial1.available() > 0) {
        while (Serial.available() > 0) {
            Serial.print(Serial.read());
        }   
    }
}
