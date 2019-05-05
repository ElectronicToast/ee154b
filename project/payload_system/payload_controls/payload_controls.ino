#include <math.h>
#include <SPI.h>
#include <SD.h>

// Pins, hardware
// Chip select for SD card
int CS = 4;
// Thermistors
int therm1 = A3;
int therm2 = A4;
// Indicator LEDs
int SDinitLED;
int LKMcommLED;
int altitudeCalibratedLED;
int readingThermistorsLED;
int allSystemsLED;
int launchSwitch;

// Global variables
float initPressure;
unsigned long lastPacemaker;
unsigned long lastGroundComm;
unsigned long launchTime;
bool autonomousMode = 0;
bool tempConcern = 0;
bool tempFreakOut = 0;
bool launched = 0;
bool doorDeployed = 0;

// Other global variables
int pacemakerPeriod = 60000;
int groundCommPeriod = 60000;
int doorTimeout;


void setup() {
  // Set up indicator LEDs
  pinMode(SDinitLED, OUTPUT);
  pinMode(LKMcommLED, OUTPUT);
  pinMode(altitudeCalibratedLED, OUTPUT);
  pinMode(readingThermistorsLED, OUTPUT);
  
  // Initialize UART 
  Serial.begin(9600);
  // Initialize SD card, 5 min timeout
  bool SDinit = initializeSDcard(300000);
  if(SDinit){
    digitalWrite(SDinitLED, HIGH);
  }
  // Make sure we're talking to the LKM
  bool LKMcomm = checkLKMcomm();
  if(LKMcomm){
    digitalWrite(LKMcommLED, HIGH);
  }
  // Calibrate the current altitude to 0 +- 5, averaging over 3 readings
  bool altitudeCalibrated = calibrateAltitude(3, 5);
  if(altitudeCalibrated){
    digitalWrite(altitudeCalibratedLED, HIGH);
  }
  bool readingThermistors = checkThermistor(therm1, -20, 50) && checkThermistor(therm2, -20, 50); 
  if(readingThermistors){
    digitalWrite(readingThermistorsLED, HIGH);
  }

  // Make sure all the systems we've checked are okay
  // I'm making this its own bool so that if we decide we want to do something (like run again
  // or whatever, we can just grab the allSystems bool
  bool allSystems = SDinit && LKMcomm && altitudeCalibrated && readingThermistors;
  if(allSystems){
    digitalWrite(allSystemsLED, HIGH);
  }
}

void loop() {
  if(! launched && digitalRead(launchSwitch)){
    launchTime = millis();
    launched = true;
  }
  pacemakerIfNeeded(pacemakerPeriod);
  burnIfNeeded(launchTime, doorTimeout);
  controlTemps();
}


// Helper functions

void takeVitals(){
  // THIS SHOULD NOT ACTUALLY BE VOID but I haven't figured out how to deal with the returns yet
  // Could make it return a pointer to an array of crap?
  // Could make a global array vitals[] and just write to that?
  // Yike?
  Serial.write("$STAT");
  while(Serial.available()){
    // We need to do something with this but I don't know what
    Serial.read();
  }
}

void pacemaker(){
  // should this actually be void? Do we want to return a bool if it's successful?
  // call takeVitals();
  // writeVitals() ?
}

boolean pacemakerIfNeeded(int pacemakerPeriod){
  if(millis() - lastPacemaker > pacemakerPeriod){
    pacemaker();
    return 1;
  }
  return 0;
}

boolean burnIfNeeded(unsigned long launchTime, int timeout){
  // Only check if we need to burn if the door hasn't already been deployed
  if(! doorDeployed){
    if(findAltitude > 100 || millis() - launchTime > timeout){
      burnWire();
    }
  }
}

void burnWire(){
  // TODO
  // Not actually even sure if this should be a void or if we can somehow return a bool to check that it was successful?
}

void handleGroundCommand(){
  // Almost certainly not void
  // Set time of last ground command
  // Write to log
  // Actually deal with the command
  // Steal the ground people's radio code for this
}

void controlTemps(){
  // TODO
  // Need to talk to other group about their controls, we can steal whoever's is best
  // set warnings if we need
  // Set duty cycle, etc
}

float findAltitude(){
  /*This is a vaguely sketchy formula that I got from
   * https://keisan.casio.com/has10/SpecExec.cgi?id=system/2006/1224585971
   * and then proceded to make the assumption that we could change our reference point to the launch 
   * altitude (instead of sea level) by using our starting pressure as Po
   */
  float pressure;
  Serial.write("$PRES");
  if(Serial.available()){
    pressure = Serial.read();
    // This is assuming the external temp is 15C; we should probably add an extra thermistor to check this
  }
    float temp = 15.0;
    float h = (pow((initPressure/pressure), 1.0/5.257) - 1) * (temp + 273.15) / .0065;
    return h;
}

bool calibrateAltitude(int nTimes, int altitudeError){
  float pressure = 0;
  for(int i = 0; i < nTimes; i++){
    Serial.write("$PRES");
    if(Serial.available()){
      pressure += Serial.read();
    }
  }
  // Take the average pressure
  initPressure = pressure / nTimes;
  // We probably can't check that the measured altitude is exactly 0.00...
  if(initPressure > 0 && -altitudeError < findAltitude() && findAltitude() < altitudeError){
    // Probably add a (generous) expected range once we know units, etc
    return 1;
  } else {
    return 0;
  }
}

bool initializeSDcard(int timeout){
  SD.begin(CS);
  // Wait for it to initialize
  while(!SD.begin(CS)){
    if(millis() > timeout){
      return 0;
    } else {
      return 1;
    }
  }
}

bool checkLKMcomm(){
  // TODO
  // Take vitals, make sure it returns numbers
  return 0;
}

bool checkThermistor(int thermistorPin, int lowerBound, int upperBound){
  double temp = readThermistor(thermistorPin);
  if(temp > lowerBound && temp < upperBound){
    return 1;
  } else {
    return 0;
  }
}

double readThermistor(int thermistorPin) {

// WE FOUND A MISTAKE IN THIS SOMEWHERE, PLS CHECK TO MAKE SURE THIS IS THE LATEST VERSION
  
  // function to read the temperature
  // input: int thermistorPin is an analog pin number (eg A0) 
  // output: double representing the temperature in Celsius
  
  // thermistorPin needs to be an analog pin
  // Connect the thermistor in series with another resistor if value Ro, with thermistor closer to ground
  float Ro = 10000.0;
  // Using Vcc = 5V
  float Vcc = 5;
  // analogRead returns an int in [0, 1023], so it needs to be scaled to volts
  float reading = analogRead(thermistorPin);
  float Vthermistor = (5 / 1023.0) * reading;
  // Now for the real question, can I use Ohm's law?
  float Rthermistor = Vthermistor * Ro / (Vcc - Vthermistor);

  // Using some model with some values I found in the NTCLE100E3 data sheet
  // number 9
  float A = 3.354016 * pow(10,-3);
  float B = 2.569850 * pow(10,-4);
  float C = 2.620131 * pow(10,-6);
  float D = 6.383091 * pow(10,-8);
  float Rref = 10000.0;
  float temp = 1.0 / (A + B * log( Rref/Rthermistor) + C * pow(log( Rref/Rthermistor), 2) + D * pow(log( Rref/Rthermistor), 3));
  // calibration factor of 18 and K -> C
  temp = temp - 273;
  // C -> F
  //temp = 9/5 * temp + 32;
  return temp; // in Celsius
}
