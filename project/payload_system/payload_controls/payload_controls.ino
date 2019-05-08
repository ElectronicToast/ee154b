#include <SD.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>

#define DOOR_ALTITUDE 95
#define TARGET_TEMP 30
#define TEMP_TOLERANCE 1

// Pins, hardware
// Burn Door Deploy 
int BDD = 4;
// thermal control
int heater = 6;
// Chip select for SD card
int CS = 7;
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
char delim = ',';
char terminator = ';';


// Temp control stuff
class averagingArray{
  public:
  static const int arraySize = 100;
  int data[arraySize];
  int iEnd = 0;
  long arraySum = 0;
  averagingArray(){
    for(int i = 0; i < arraySize; i++){
      data[i] = 0;
    }
  }
  long pop(){
    long foundValue = data[iEnd];
    arraySum -= foundValue;
    return foundValue;
  }
  
  long addNew(long newData){
//    if(iEnd == 0){
//      Serial.println();
//      Serial.println("Array Filled");
//      Serial.println();
//    }
    long prevVal = -1;
    // Update the end
    iEnd++;
    iEnd = iEnd % arraySize;
    
    // Pop off the old value, add the new one
    prevVal = pop();
    data[iEnd] = newData;
    arraySum += newData;
//    Serial.print("arraySum = ");
//    Serial.println(arraySum);
    return prevVal;
  }
  
  float average(){
    // This is supposed to return a float, but it will return an int...
    return arraySum / arraySize;
  }

  int lastVal(){
    return data[iEnd];
  }

  int lastDeltaY(){
    return data[iEnd] - data[(iEnd - 1 + arraySize) % arraySize];
  }
  
};

averagingArray data;

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
  burnIfNeeded(doorTimeout);
  controlTemps(TARGET_TEMP, TEMP_TOLERANCE);
  handleGroundCommand();
}


// Helper functions
void recordVitals(String event){
  Serial.write("$STAT;");
  String telem = "";
  delay(100);
  if (Serial.available()) {
    telem = Serial.readStringUntil(terminator);
  }
  
  // open the file.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write the time and vitals to it:
  if (dataFile) {
    dataFile.println(millis() + ',' + telem + ',' + event);
    dataFile.close();
  }

}

boolean pacemakerIfNeeded(int pacemakerPeriod){
  if(millis() - lastPacemaker > pacemakerPeriod){
    recordVitals("");
    return 1;
  }
  return 0;
}

boolean burnIfNeeded(int timeout){
  // Only check if we need to burn if the door hasn't already been deployed
  if(! doorDeployed){
    if(findAltitude() > DOOR_ALTITUDE || millis() - launchTime > timeout){
      burnWire();
      doorDeployed = 1;
    }
  }
}

void burnWire(){
  // TODO
  digitalWrite(BDD, HIGH);
  recordVitals("Door: deployed");
}

void controlTemps(float target, float err_tolerance){
  // TODO
  // which temp reading am i supposed to use??? we doing temp of LKM processor?
  float temp = readThermistor(therm1);
  Serial.write("$TEMP;");
  if (Serial.available()) {
    Serial.readStringUntil(delim); // should be #TEMP,
    temp = Serial.readStringUntil(terminator).toFloat();
  }
  float error = target - temp;
  data.addNew(error);
  // prolly need to somehow record time elapsed between thermistor readings
  // also need to record error into data structure 
  // record any temperatures outside of a tolerable range
  if (error > err_tolerance) {
    recordVitals("WARNING: temp too low");
  }
  else if (error < -err_tolerance) {
    recordVitals("WARNING: temp too high");
  }
  //need to tune this as well... oh boy 
  analogWrite(heater, 255.0 / 100.0 * PID(0.5, 0.1, 0)); 
}

void handleGroundCommand(){
  // Almost certainly not void
  // Set time of last ground command
  // Write to log
  // Actually deal with the command
  // Steal the ground people's radio code for this
}

float findAltitude(){
  /*This is a vaguely sketchy formula that I got from
   * https://keisan.casio.com/has10/SpecExec.cgi?id=system/2006/1224585971
   * and then proceded to make the assumption that we could change our reference point to the launch 
   * altitude (instead of sea level) by using our starting pressure as Po
   */
  float pressure;
  Serial.write("$PRES;");
  if (Serial.available()) {
    Serial.readStringUntil(delim); // should be #PRES,
    pressure = Serial.readStringUntil(terminator).toFloat();
  }
  
  // This is assuming the external temp is 15C; we should probably add an extra thermistor to check this
  // make sure pressure is in hPa?
  // this returns height in meters... will need to convert to feet
  float temp = 15.0;
  float h = (pow((initPressure/pressure), 1.0/5.257) - 1) * (temp + 273.15) / .0065;
  return h;
}

bool calibrateAltitude(int nTimes, int altitudeError){
  float pressure = 0;
  for(int i = 0; i < nTimes; i++){
    Serial.write("$PRES;");
    delay(100);
    if(Serial.available()){
      Serial.readStringUntil(delim); //should be #PRES,
      pressure += Serial.readStringUntil(terminator).toFloat();
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
  float temp = readThermistor(thermistorPin);
  if(temp > lowerBound && temp < upperBound){
    return 1;
  } else {
    return 0;
  }
}

float readThermistor(int thermistorPin) {
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

double PID(float Pcoeff, float Icoeff, float Dcoeff){
 float proportionalControl = Pcoeff * data.lastVal();
 float integralControl = Icoeff * data.average();
 float derivativeControl = Dcoeff * data.lastDeltaY() / delayTime;
 float controlRaw = proportionalControl + integralControl + derivativeControl;
 return max(min(controlRaw, 1), 0) * 100;
//  return 0.0;
}
