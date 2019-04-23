#include <EEPROM.h>


// Character constants
#define     CHAR_NRST_RESET   '1'
#define     CHAR_A      0x41
#define     CHAR_Z      0x5A
#define     CHAR_Zp     0x5B
#define     CHAR_a      0x61
#define     CHAR_z      0x7A
#define     CHAR_zp     0x7B


// System constants
#define     BAUDRATE    9600
#define THIRTY_S_MS     30000


// Logarithmic Thermistor calibration
#define TMST_LOG_SLOPE -22.025
#define TMST_LOG_INT 228.406

// ADC Resolution
const double ADC_RES = 1024.0;


// EEPROM constants
#define INT_SIZE          2
// Size (bytes) of block of EEPROM for storing each temp reading
#define TEMP_BLOCK_SIZE   6
#define TEMP_2BLOCK_SIZE  12

// Temperature logging constants
#define TEMP_LOG_START_CHAR     '$'     // Logging start indicator
#define TEMP_LOG_END_CHAR       '%'     // Logging end indicator

// Temperature readback constants
#define TEMP_SEP_CHAR     ','     // Value separator
#define TEMP_EOM_CHAR     '#'     // Readback terminator


const int TMST_PIN = 0;
char c_rx;

const int resetaddr = 0;          // Reset stored at address 0
#define TEMP_BLOCK_START      2
int tempaddr = 2;                 // Blocks start at address 2
const int lengthaddr = 510;       // Most recent block address stored at 511 | 510

char input;

unsigned int resetcount;          // EEROM #resets


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
        else if(input == '0'){ //increment string
            while(Serial.available()>0){
                c_rx = Serial.read(); //next character
                Serial.print(shiftup(c_rx));
                delay(5);
            }
        }
        //***************************  
        else if(input == '1'){ //decrement string
            while(Serial.available()>0){
                c_rx = Serial.read(); //next character
                Serial.print(shiftdown(c_rx));
                delay(5);
            }
        }
         //***************************  
         else if(input == '2'){ //read thermistor in deg C
            double tmst_1_raw = (double) analogRead(TMST_PIN);
            double tempc = calibrated(tmst_1_raw);
            Serial.print(tempc);
        }
        //***************************  
        else if(input == '3'){ //read thermistor in deg F
            double tmst_1_raw = (double) analogRead(TMST_PIN);
            double tempc = calibrated(tmst_1_raw); 
            double tempf = tempc * (9/5) + 32;
            Serial.print(tempf);    
        }
        //***************************  
        else if(input == '4'){ //do a reset
            EEPROM.get(resetaddr, resetcount);
            EEPROM.write(resetaddr, resetcount + 1);
            Serial.print(resetcount);     // Print current # resets
            delay(5);     //prevent transmission corruption
            resetFunc();  //call reset
        }
         //*************************** 
         else if(input == '5'){ //record timestamp and temp in EEPROM every 30 s
            while(Serial.read()!='6'){
                int t = millis();
                double tmst_1_raw = (double) analogRead(TMST_PIN);
                double tempc = calibrated(tmst_1_raw);
                int writeaddr = tempaddr;

                // Send start indicator
                Serial.print(TEMP_LOG_START_CHAR);
                
                // While in block range
                //    If all of EEPROM is used, not stored
                if (writeaddr <= lengthaddr - TEMP_BLOCK_SIZE) {
                    EEPROM.put(writeaddr, t);       
                    EEPROM.put(writeaddr + INT_SIZE, tempc);
                    EEPROM.put(lengthaddr, writeaddr);
                    tempaddr += TEMP_BLOCK_SIZE;      // Advance to next address
                }
                
                delay(1000);
                //delay(THIRTY_S_MS);  
            }    
            // Send end indicator
            Serial.print(TEMP_LOG_START_CHAR);
        }
         //***************************
        else if(input == '7'){ //read timestamp and temp
            int ptr = TEMP_BLOCK_START;
            int writeaddr;
            EEPROM.get(lengthaddr, writeaddr);
            while(ptr <= writeaddr){
                int t;
                double temp;
                EEPROM.get(ptr, t);
                EEPROM.get(ptr + INT_SIZE, temp);
                
                Serial.print(t);
                Serial.print(TEMP_SEP_CHAR);
                Serial.print(temp);
                Serial.print(TEMP_SEP_CHAR);
                
                ptr += TEMP_BLOCK_SIZE;           // Increment by block size
            }    
            Serial.print(TEMP_EOM_CHAR);
        }
        //***************************
        else if(input == '8'){ //read every other timestamp and temp
            int ptr = TEMP_BLOCK_START;
            int writeaddr = EEPROM.read(lengthaddr);
            while(ptr <= writeaddr){
                int t;
                double temp;
                EEPROM.get(ptr, t);
                EEPROM.get(ptr + INT_SIZE, temp);
                
                Serial.print(t);
                Serial.print(TEMP_SEP_CHAR);
                Serial.print(temp);
                Serial.print(TEMP_SEP_CHAR);
                
                ptr += TEMP_2BLOCK_SIZE;    // Increment by twice block size
            }  
            Serial.print(TEMP_EOM_CHAR);
        }
    //***************************
    }
}

char shiftup (char c) {
    char c_shift = c + 1;
    if (c == CHAR_Z)
        c_shift = CHAR_A;
    else if (c == CHAR_z)
        c_shift = CHAR_a;
    return c_shift;
}

char shiftdown (char c) {
    char c_shift = c - 1;
    if (c == CHAR_A)
        c_shift = CHAR_Z;
    else if (c == CHAR_a)
        c_shift = CHAR_z;
    return c_shift;
}

double calibrated(double x){
    double r = 10000.0 * x / (ADC_RES - x);
    double calibrated = TMST_LOG_SLOPE * log(r) + TMST_LOG_INT;
    return calibrated;
}
