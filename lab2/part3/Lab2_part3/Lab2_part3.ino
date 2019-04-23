#include <EEPROM.h>

/**
 * This Arduino sketch allows an external "commander" - a laptop with a UI 
 * program - to command various actions including UART loopback tests,
 * thermistor reading/logging, and EEPROM storage, by the UART interface
 * to an Arduino Uno.
 * 
 * The following actions may be commanded by sending the respective 
 * character to the Arduino over UART:
 *      - 0     increment a payload string (shift characters up)
 *      - 1     decrement a payload string (shift characters down)
 *      - 2     read a thermistor in Celsius
 *      - 3     read a thermistor in Farenheit
 *      - 4     reset the Arduino (software based) and send number of resets 
 *      - 5     begin logging time-tagged thermistor values per 30 seconds
 *              to Arduino EEPROM, in Celsius
 *      - 6     stop logging 
 *      - 7     read back logged times and temperatures
 *      - 8     read back every other time, temperature 
 *      - !     reset reset count in EEPROM 
 *
 * The Arduino is reset by software, setting the program counter to address 0.
 *
 * The 1024 bytes of EEPROM on the Arduino Uno are organized as follows 
 *
 *      | 0      1 | 2                                    1021 | 1022 1023 |
 *        # resets     6-byte wide time, temp readings       addr of last read
 *
 * This permits 170 time-tagged readings to be stored. If more readings than
 * available EEPROM space is collected, they are not stored.
 *
 * The thermistor is connected between Arduino analog pin 0 and ground, with a 
 * 10k resistor pullup to 5V.
 */


// Character constants
#define     CHAR_NRST_RESET     '!'
#define     CHAR_A              0x41
#define     CHAR_Z              0x5A
#define     CHAR_Zp             0x5B
#define     CHAR_a              0x61
#define     CHAR_z              0x7A
#define     CHAR_zp             0x7B


// System constants
#define     BAUDRATE        9600
#define     THIRTY_S_MS     30000


// Logarithmic Thermistor calibration
#define     TMST_LOG_SLOPE  -22.025
#define     TMST_LOG_INT    228.406

// ADC Resolution
const double ADC_RES = 1024.0;


// EEPROM constants
#define     INT_SIZE            2
// Size (bytes) of block of EEPROM for storing each temp reading
#define     TEMP_BLOCK_START    2
#define     TEMP_BLOCK_SIZE     6
#define     TEMP_2BLOCK_SIZE    12

// Temperature logging constants
#define     TEMP_LOG_START_CHAR     '$'     // Logging start indicator
#define     TEMP_LOG_END_CHAR       '%'     // Logging end indicator

// Temperature readback constants
#define     TEMP_SEP_CHAR     ','           // Value separator
#define     TEMP_EOM_CHAR     '#'           // Readback terminator


// Thermistor analog input pin
const int TMST_PIN = 0;


char c_rx;

const int resetaddr = 0;          // Reset stored at address 0
int tempaddr = 2;                 // Blocks start at address 2
const int lengthaddr = 1022;      // Most recent block address location

char input;

unsigned int resetcount;          // EEPROM #resets


void setup() {
  Serial.begin(BAUDRATE);
  pinMode(TMST_PIN, INPUT);
  // put your setup code here, to run once:
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void loop() {

    if(Serial.available()){
        input = Serial.read(); //first character
        delay(5);
        
        //****************************
        // Clear reset counts if 'CHAR_NRST_RESET' entered
        if(input == CHAR_NRST_RESET){ 
            EEPROM.write(resetaddr, 0);
            EEPROM.write(resetaddr + 1, 0);
        }
        //****************************   
        // Increment string
        else if(input == '0'){
            while(Serial.available()>0){
                c_rx = Serial.read(); //next character
                Serial.print(shiftup(c_rx));
                delay(5);
            }
        }
        //***************************  
        // Decrement string
        else if(input == '1'){
            while(Serial.available()>0){
                c_rx = Serial.read(); //next character
                Serial.print(shiftdown(c_rx));
                delay(5);
            }
        }
        //***************************  
        // Read thermistor in deg C
        else if(input == '2'){ 
            double tmst_1_raw = (double) analogRead(TMST_PIN);
            double tempc = calibrated(tmst_1_raw);
            Serial.print(tempc);
        }
        //***************************  
        // Read thermistor in deg F
        else if(input == '3'){
            double tmst_1_raw = (double) analogRead(TMST_PIN);
            double tempc = calibrated(tmst_1_raw); 
            double tempf = tempc * (9/5) + 32;
            Serial.print(tempf);    
        }
        //***************************  
        // Reset the Arduino and keep track of # resets in low 2 bytes of 
        // EEPROM
        else if(input == '4'){
            EEPROM.get(resetaddr, resetcount);
            EEPROM.write(resetaddr, resetcount + 1);
            Serial.print(resetcount);     // Print current # resets
            delay(5);                     // Prevent transmission corruption
            resetFunc();                  // Call reset
        }
        //*************************** 
        // Record timestamp and temp in EEPROM every 30 s
        else if(input == '5'){ 
            // Send start indicator
            Serial.print(TEMP_LOG_START_CHAR);
            
            // Stop reading if stop character received
            while(Serial.read()!='6'){
                // Read current time and temperature in Celsius
                int t = millis();
                double tmst_1_raw = (double) analogRead(TMST_PIN);
                double tempc = calibrated(tmst_1_raw);
                
                int writeaddr = tempaddr;       // Current write address
                
                // While in block range
                //    If all of EEPROM is used, don't store
                if (writeaddr <= lengthaddr - TEMP_BLOCK_SIZE) {
                    EEPROM.put(writeaddr, t);       
                    EEPROM.put(writeaddr + INT_SIZE, tempc);
                    EEPROM.put(lengthaddr, writeaddr);
                    
                    tempaddr += TEMP_BLOCK_SIZE;      // Advance to next block
                }
                
                delay(THIRTY_S_MS);  
            }    
            // Send end indicator
            Serial.print(TEMP_LOG_END_CHAR);
        }
        //***************************
        // Readback all timestamps and temperature readings
        else if(input == '7'){
            int ptr = TEMP_BLOCK_START;     // Init pointer at start of blocks
            int writeaddr;
            
            // Fetch most recently used block
            EEPROM.get(lengthaddr, writeaddr);
            
            while(ptr <= writeaddr){
                // Read the time and temperature of the current block
                int t;
                double temp;
                EEPROM.get(ptr, t);
                EEPROM.get(ptr + INT_SIZE, temp);
                
                // Send the values over UART with appropriate separtion
                Serial.print(t);
                Serial.print(TEMP_SEP_CHAR);
                Serial.print(temp);
                Serial.print(TEMP_SEP_CHAR);
                
                ptr += TEMP_BLOCK_SIZE;     // Go to next block
            }    
            Serial.print(TEMP_EOM_CHAR);    // Send end of read character
        }
        //***************************
        // Readback every other all timestamps and temperature readings
        else if(input == '8'){ 
            int ptr = TEMP_BLOCK_START;     // Init pointer at start of blocks
            int writeaddr;
            
            // Fetch most recently used block
            EEPROM.get(lengthaddr, writeaddr);
            
            while(ptr <= writeaddr){
                // Read the time and temperature of the current block
                int t;
                double temp;
                EEPROM.get(ptr, t);
                EEPROM.get(ptr + INT_SIZE, temp);
                
                // Send the values over UART with appropriate separtion
                Serial.print(t);
                Serial.print(TEMP_SEP_CHAR);
                Serial.print(temp);
                Serial.print(TEMP_SEP_CHAR);
                
                ptr += TEMP_2BLOCK_SIZE;    // Go to every other block
            }  
            Serial.print(TEMP_EOM_CHAR);    // Send end of read character
        }
    //***************************
    }
}

/* This function shifts a character 'c' "up" by one (e.g. a -> b, Z -> A).
 * Valid character input 'c' is assumed.
 */
char shiftup (char c) {
    char c_shift = c + 1;
    if (c == CHAR_Z)
        c_shift = CHAR_A;
    else if (c == CHAR_z)
        c_shift = CHAR_a;
    return c_shift;
}

/* This function shifts a character 'c' "down" by one (e.g. a -> z, Z -> Y).
 * Valid character input 'c' is assumed.
 */
char shiftdown (char c) {
    char c_shift = c - 1;
    if (c == CHAR_A)
        c_shift = CHAR_Z;
    else if (c == CHAR_a)
        c_shift = CHAR_z;
    return c_shift;
}

/* Given a raw thermistor reading 'x' in double format, this function 
 * computes the temperature in Celsius based on a log calibration curve.
 */
double calibrated(double x){
    double r = 10000.0 * x / (ADC_RES - x);
    double calibrated = TMST_LOG_SLOPE * log(r) + TMST_LOG_INT;
    return calibrated;
}
