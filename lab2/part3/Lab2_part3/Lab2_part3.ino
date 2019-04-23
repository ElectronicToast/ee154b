#define     BAUDRATE    9600

#define     CHAR_A      0x41
#define     CHAR_Z      0x5A
#define     CHAR_Zp     0x5B
#define     CHAR_a      0x61
#define     CHAR_z      0x7A
#define     CHAR_zp     0x7B
#define     CHAR_0      0x30
#define     CHAR_9      0x39
#include <EEPROM.h>

// Logarithmic Thermistor calibration
#define TMST_LOG_SLOPE -22.025
#define TMST_LOG_INT 228.406

// ADC Resolution
const double ADC_RES = 1024.0;

const int TMST_PIN = 0;
char c_rx;

int resetaddr = 0;
int tempaddr = 1;
int lengthaddr = 511;

char input;

byte resetcount;     // EEROM data in bytes


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
        // Clear reset counts
        if(input == '!'){ 
            EEPROM.write(resetaddr, 0);
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
            resetcount = EEPROM.read(resetaddr);
            EEPROM.write(resetaddr, resetcount + 1);
            Serial.print(resetcount);
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
          
                EEPROM.write(writeaddr, t);
                EEPROM.write(writeaddr+1, tempc);
                EEPROM.write(lengthaddr, writeaddr);\
                delay(30000);  
            }    
        }
         //***************************
        else if(input == '7'){ //read timestamp and temp
            int ptr = 1;
            int writeaddr = EEPROM.read(lengthaddr);
            while(ptr <= writeaddr){
                EEPROM.read(ptr);
                EEPROM.read(ptr+1);
                ptr = ptr + 2;  
            }    
        }
        //***************************
        else if(input == '8'){ //read every other timestamp and temp
            int ptr = 1;
            int writeaddr = EEPROM.read(lengthaddr);
            while(ptr <= writeaddr){
                EEPROM.read(ptr);
                EEPROM.read(ptr+1);
                ptr = ptr + 4;  
          }  
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
