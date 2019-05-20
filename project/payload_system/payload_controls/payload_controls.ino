#include <SD.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>

#define DOOR_ALTITUDE 95
#define TARGET_TEMP 30
#define TEMP_TOLERANCE 1
#define MAX_RESTART_TEMP 30
#define USB_BAUD 9600
#define LKM_DEFAULT_BAUD 9600
#define LKM_STARTUP_TIME 35000
#define DEFAULT_DEMAND_N 50
#define SD_TIMEOUT 10000
#define STAT_BUFFER_TIME 300
#define SERIAL_TIMEOUT 3000
#define COOLTIME_TIMEOUT 300000

#define BAD_CMD -100
#define BAD_VAL -200
#define MISMATCH -300
#define BAD_STAT -400

#define ADC_N_BITS      10
#define ADC_MAX_VAL     1023
#define ADC_VREF        3.3

#define SHUNT_RES_OHM   3.0
#define SHUNT_SCALE_H   4.13832853025937
#define SHUNT_SCALE_L   4.323499491353

#define PWR_INDEX  0
#define PULS_INDEX 1
#define DATA_INDEX 2
#define VOLT_INDEX 3
#define PRES_INDEX 4
#define TEMP_INDEX 5
#define MOTR_INDEX 6

// Debugging modes
bool LKMStillOn = true;
bool autoFreakAndTurnOffIntervalEnabled = false;


// Pins, hardware
// Burn Door Deploy 
int BDD = 4;
// thermal control
int heater = 6;
// Chip select for SD card
int CS = 8;
// Thermistors
int therm1 = A0;
int therm2 = A3;
// Voltage/Current Sense
int shunt_h = A2;     // Not correct but gives >0 most of the time
int shunt_l = A1;
// Indicator LEDs
int SDinitLED = 3;
int LKMcommLED = 22;
int altitudeCalibratedLED = 24;
int readingThermistorsLED = 26;
int allSystemsLED = 28;
int launchedLED = 30;
int launchSwitch = 2;

// Global variables
float initPressure;
unsigned long lastPacemaker = 0;
unsigned long lastGroundComm;
unsigned long launchTime;
unsigned long lastTempControl = 0;
unsigned long timeEnteredAutonomous;
unsigned long firstKillSignalTime;
bool autonomousMode = 0;
bool tempConcern = 0;
bool tempFreakOut = 0;
bool launched = 0;
bool doorDeployed = 0;
bool emergencyKill = 0;
bool emergencyKillFirstSignal = 0;
float PID_kP = .5;
float PID_kI = .1;
float PID_kD = 0;

// Other global variables
int pacemakerPeriod = 60000;
int groundCommPeriod = 300000;
int burnTime = 60000;
int doorTimeout = 6000;
int LKMsetupTimeout = 100000;
int killSignalInterval = 60000;
int autoFreakAndTurnOffInterval = 600000;
char delim = ',';
char terminator = ';';

// global arrays to store telem 
// [PWR, PULS, DATA, VOLT, PRES, TEMP, MOTR]
enum Command {PWR, PULS, DATA, VOLT, PRES, STAT, MOTR, KP, KI, KD};
float expected_val[] = {1.0, 0.0, 9600, 5.0, 1000.0, 27.0, 30.0};
float stats[7];

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
    long prevVal = -1;
    // Update the end
    iEnd++;
    iEnd = iEnd % arraySize;
    
    // Pop off the old value, add the new one
    prevVal = pop();
    data[iEnd] = newData;
    arraySum += newData;
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


//
//
//
//
//
//    SETUP BEGIN
//
//
//
//
//


void setup() {  
  // Set up indicator LEDs
  pinMode(SDinitLED, OUTPUT);
  pinMode(LKMcommLED, OUTPUT);
  pinMode(altitudeCalibratedLED, OUTPUT);
  pinMode(readingThermistorsLED, OUTPUT);
  pinMode(allSystemsLED, OUTPUT);
  pinMode(launchedLED, OUTPUT);
  pinMode(BDD, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(launchSwitch, INPUT);
  digitalWrite(BDD, LOW);
  digitalWrite(SDinitLED, LOW);
  digitalWrite(LKMcommLED, LOW);
  digitalWrite(altitudeCalibratedLED, LOW);
  digitalWrite(readingThermistorsLED, LOW);
  digitalWrite(allSystemsLED, LOW);
  digitalWrite(launchedLED, LOW);
  digitalWrite(CS, LOW);

  // Blink to indicate initialization
  digitalWrite(LKMcommLED, HIGH);
  delay(100);
  digitalWrite(LKMcommLED, LOW);
  delay(100);
  digitalWrite(LKMcommLED, HIGH);
  delay(100);
  digitalWrite(LKMcommLED, LOW);
  delay(100);
  
  // Initialize UARTs 
  Serial.begin(9600);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial2.begin(9600);
  Serial2.setTimeout(SERIAL_TIMEOUT);
  if(! LKMStillOn){
    Serial1.begin(LKM_DEFAULT_BAUD);
    Serial1.setTimeout(SERIAL_TIMEOUT);
  }
  if(LKMStillOn){
    Serial1.begin(2400);
    Serial1.setTimeout(SERIAL_TIMEOUT);
  }

  Serial2.println("Startup\n");

  digitalWrite(LKMcommLED, HIGH);
  delay(500);
  digitalWrite(LKMcommLED, LOW);
  delay(250);
  digitalWrite(LKMcommLED, HIGH);
  delay(500);
  digitalWrite(LKMcommLED, LOW);
  delay(250);
  digitalWrite(LKMcommLED, HIGH);
  delay(500);
  digitalWrite(LKMcommLED, LOW);

  Serial2.write("Hello");
  
  // Initialize SD card
  bool SDinit = initializeSDcard(SD_TIMEOUT);
  
  if(SDinit){
    digitalWrite(SDinitLED, HIGH);
    Serial.println("SDinit true");
  }
  // Make sure we're talking to the LKM
  // Delay long enough for the LKM to start up
  if(! LKMStillOn){
    Serial.print("Waiting for LKM startup...");
    delay(LKM_STARTUP_TIME);
  }
  // Change baud rate
  // Maybe we should check if it's done starting up first?
  // UNCOMMENT FOR FLIGHT
  if(! LKMStillOn){
    Serial2.println(Serial1.readStringUntil('\n'));
  }

  // UNCOMMENT FOR FLIGHT
  if(! LKMStillOn){
    Serial2.println("Lowering LKM baud rate...");
    lowerBaudRate(2400);
  }

  Serial.println("Done.");
  Serial.println("Connecting to LKM...");
  
  bool LKMcomm = 0;
  unsigned long startLKMtestTime = millis();
  while(! LKMcomm && (millis() - startLKMtestTime < LKMsetupTimeout)){
    LKMcomm = checkLKMcomm(10);
  }
  if(LKMcomm){
    digitalWrite(LKMcommLED, HIGH);
    Serial.println("Connected to LKM");
    Serial.println("Should light LKMcommLED");
  }

  Serial.print("Calibrating sensors...");
    
  // Calibrate the current altitude to 0 +- 5, averaging over 3 readings
  Serial.println("Trying to calibrate altitude");
  bool altitudeCalibrated = calibrateAltitude(3, 5);
  if(altitudeCalibrated){
    digitalWrite(altitudeCalibratedLED, HIGH);
    Serial.println("altitude calibrated");
    Serial.println("Should light altitudeCalibratedLED");
  }
  bool readingThermistors = checkThermistor(therm1, -20, 50) && checkThermistor(therm2, -20, 50); 
  if(readingThermistors){
    digitalWrite(readingThermistorsLED, HIGH);
    Serial.println("reading thermistors");
    Serial.println("Should light readingThermistorsLED");
  }

  Serial.println("Done.");
  
  Serial2.println("Turning off LKM heater...");
  // Turn the heater off
  Serial1.print("$PULS, 0;");
  Serial.println("Done.");

  //////////////////////////////////////
  //Serial.println("Reading current");
  //Serial.println(readCurrent());
  //delay(5000);
  //Serial.println(readCurrent());
  //delay(5000);
  //Serial.println(readCurrent());
  ///////////////////////////////////////

  Serial2.print("Checking systems status...");
  
  // Make sure all the systems we've checked are okay
  // I'm making this its own bool so that if we decide we want to do something (like run again
  // or whatever, we can just grab the allSystems bool
  bool allSystems = SDinit && LKMcomm && altitudeCalibrated && readingThermistors;
  if(allSystems){
    digitalWrite(allSystemsLED, HIGH);
    Serial2.print("\tGO\t");
  }

  Serial1.flush();
  
  Serial2.println("Finished startup function");
}

void loop() {
  if(! launched && digitalRead(launchSwitch)== LOW){
    launchTime = millis();
    launched = true;
    digitalWrite(launchedLED, HIGH);
    Serial.println("Launch button pushed");
    recordVitals("Event: launched");
  }
  pacemakerIfNeeded(pacemakerPeriod);
  burnIfNeeded(doorTimeout);
  controlTemps(TARGET_TEMP, TEMP_TOLERANCE);
  handleGroundCommand();
  monitorVals();
  if(millis() - lastGroundComm > groundCommPeriod){
    autonomousMode = true;
    Serial2.println("Yo talk to us");
    recordVitals("Entered autonomous mode");
    timeEnteredAutonomous = millis();
  }
  while(autonomousMode && launched){
    pacemakerIfNeeded(pacemakerPeriod);
    burnIfNeeded(doorTimeout);
    controlTemps(TARGET_TEMP, TEMP_TOLERANCE);
    monitorVals();
    Serial2.println("Pls I'm lonely");
    Serial2.print("Entered autonomous mode ");
    Serial2.print((millis() - timeEnteredAutonomous) / 1000);
    Serial2.println(" seconds ago");
    if((millis() - timeEnteredAutonomous > autoFreakAndTurnOffInterval) && autoFreakAndTurnOffIntervalEnabled){
      // Enter autoFreakAndTurnOffMode
      recordVitals("Entering autoFreakAndTurnOffMode");
      unsigned long timeoutTime = millis() + COOLTIME_TIMEOUT;
      // Figure out if we're too hot; in this case, it might be worth turning the LKM off
      bool tooHot = demandVal("$TEMP;", 30) > TARGET_TEMP + TEMP_TOLERANCE;
      if(tooHot){
        recordVitals("Decided processor was too hot. Powering LKM off");
        Serial1.print("PWR,OFF;");
      }
      // figure out once we have thermistors on processor, but probably something like this
      while(readThermistor(therm1) > MAX_RESTART_TEMP && millis() < timeoutTime){
        // Communicate with ground
        Serial2.println("Communication with ground lost. Processor temp too hot. Suspected spinning. Waiting for processor to cool.");
        Serial2.print("Current temp (thermistor reading): ");
        Serial2.println(readThermistor(therm1));
        // listen to ground just in case
        handleGroundCommand();
        // Wait a second so we're not too obnoxious to ground
        delay(1000);
      }
      powerLKMon(2400);
    }
    if(handleGroundCommand()){
      autonomousMode = false;
      Serial2.print("Exited autonomous mode;");
    }
  }
  while(emergencyKill){
    handleGroundCommand();
  }
}


// Helper functions
void recordVitals(String event){
  Serial.print("Called recordVitals with event: ");
  Serial.println(event);
  Serial.flush();
  Serial1.write("$STAT;");
  String telem = "";
  delay(100);
  if (Serial1.available()) {
    telem = Serial1.readStringUntil(terminator);
    while(telem.charAt(0)=='\n' || telem.charAt(0)=='\r' ) {
    telem = telem.substring(1);
    }
  }
  Serial1.flush();
  // open the file.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write the time and vitals to it:
  if (dataFile) {
    dataFile.print(millis());
    dataFile.print(',');
    dataFile.print(telem);
    dataFile.print(',');
    dataFile.println(event);
    dataFile.close();
  }
  Serial2.print("Event: ");
  Serial2.println(event);
}

boolean pacemakerIfNeeded(int pacemakerPeriod){
//  Serial.println("Called pacemakerIfNeeded");
  if(millis() - lastPacemaker > pacemakerPeriod){
    recordVitals("Pacemaker");
    lastPacemaker = millis();
    return 1;
  }
  return 0;
}

boolean burnIfNeeded(int timeout){
//  Serial.println("Called burnIfNeeded");
  // Only check if we need to burn if the door hasn't already been deployed
  if(! doorDeployed && launched){
    if(findAltitude() > DOOR_ALTITUDE ||(millis() - launchTime > timeout)){
      Serial2.print("Burn wire activated, altitude ");
      Serial2.println(findAltitude());
      Serial.print("time ");
      Serial.print(millis() - launchTime);
      burnWire();
      doorDeployed = 1;
    }
  }
}

void burnWire(){
  Serial.println("Called burnWire");
  recordVitals("Door: burn started");
  digitalWrite(BDD, HIGH);
  delay(burnTime);
  digitalWrite(BDD, LOW);
  recordVitals("Door: deployed");
  Serial2.println("Door: deployed");
}

void controlTemps(float target, float err_tolerance){
  Serial.println("Called controlTemps");
  // Should use Klesh's function to get temp
  // which temp reading am i supposed to use??? we doing temp of LKM processor?
  float temp = readThermistor(therm1);
  // temp = readThermistor(therm1);
  Serial1.write("$TEMP;");
  temp = parseLKM();
  if (temp == BAD_VAL || temp == BAD_CMD || temp == MISMATCH) {
    return;
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
  analogWrite(heater, 255.0 / 100.0 * PID(PID_kP, PID_kI, PID_kD)); 
  lastTempControl = millis();
}


float findAltitude(){
  Serial.println("Called findAltitude");
  /*This is a vaguely sketchy formula that I got from
   * https://keisan.casio.com/has10/SpecExec.cgi?id=system/2006/1224585971
   * and then proceded to make the assumption that we could change our reference point to the launch 
   * altitude (instead of sea level) by using our starting pressure as Po
   */
  float pressure;
  pressure = demandVal("$PRES;", DEFAULT_DEMAND_N);
  // This is assuming the external temp is 15C; we should probably add an extra thermistor to check this
  // make sure pressure is in hPa?
  // this returns height in meters... will need to convert to feet
  float temp = 15.0;
  float h = (pow((initPressure/pressure), 1.0/5.257) - 1) * (temp + 273.15) / .0065;
  return h;
}

bool calibrateAltitude(int nTimes, int altitudeError){
  Serial.println("Called calibrateAltitude");
  Serial2.print("Calibrating altitude...");
  while(Serial1.available()){
    Serial.print(Serial1.read());
  }
  float pressure = 0;
  int nTries = 0;
  int maxTries = 200;
  int realMeasurements = 0;
  
  for(int i = 0; i < nTimes; i++){
    float readPressure = demandVal("$PRES;", 2 * DEFAULT_DEMAND_N);
    if(readPressure > 0){
      pressure += readPressure;
    }
  }
  // Take the average pressure
  initPressure = pressure / nTimes;
  // We probably can't check that the measured altitude is exactly 0.00...
  if(initPressure > 0 && -altitudeError < findAltitude() && findAltitude() < altitudeError){
    // Probably add a (generous) expected range once we know units, etc
    Serial2.println("Success");
    return 1;
  } else {
    Serial2.print("Altitude calibration failed. initPressure: ");
    Serial2.println(initPressure);
    return 0;
  }
}

bool initializeSDcard(int timeout){
  Serial2.print("initializing SD card...");
  SD.begin(CS);
  // Wait for it to initialize
  unsigned long startTime = millis();
  while(!SD.begin(CS)){
    if(millis() - startTime > timeout){
      Serial2.println("Failure");
      Serial2.println("Giving up on SD");
      return 0;
    }
  }
  Serial.println("SD initialized");
  return 1;
}

bool checkLKMcomm(int nTries){
  Serial2.print("Checking communications with LKM...");
  for(int i = 0; i < nTries; i++){
    Serial1.write("$STAT;");
    delay(STAT_BUFFER_TIME);
    if(parseStat() == 1.0){
      Serial2.println("Successful");
      return 1;
    }
  }
  Serial2.println("failed");
  return 0;
}

bool checkThermistor(int thermistorPin, int lowerBound, int upperBound){
  Serial2.print("Checking thermistor readings...");
  float temp = readThermistor(thermistorPin);
  if(temp > lowerBound && temp < upperBound){
    Serial2.println("Success");
    return 1;
  } else {
    Serial2.println("Failure");
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

// ------------------------------- CURRENT SHUNT -------------------------------

double readAnalogVoltage(int pin) {
  // Read and compute an analog voltage reading on `pin`
  return (double) analogRead(pin) * (double)ADC_VREF / (double)ADC_MAX_VAL;
}

double readCurrent() {
  // Read both sides of shunt resistor
  double in_h = readAnalogVoltage(shunt_h);
  double in_l = readAnalogVoltage(shunt_l);
  // Compute the current 
  return abs(SHUNT_SCALE_H * in_h - SHUNT_SCALE_L * in_l) / SHUNT_RES_OHM;
}

double PID(float Pcoeff, float Icoeff, float Dcoeff){
 float proportionalControl = Pcoeff * data.lastVal();
 float integralControl = Icoeff * data.average();
 float derivativeControl = Dcoeff * data.lastDeltaY() / (millis() - lastTempControl);
 float controlRaw = proportionalControl + integralControl + derivativeControl;
 return max(min(controlRaw, 1), 0) * 100;
//  return 0.0;
}

bool lowerBaudRate(int baud){
  Serial1.flush();
  Serial.println("Called lowerBaudRate");
  // Assume we're communicating with the LKM at 9600 to start
  // Lower baud rate using $DATA
  Serial1.print("$DATA,");
  Serial1.print(String(baud));
  Serial1.print(';');
  delay(100);
  bool error = 0;
  while(Serial1.available()){
    // If there's stuff on the serial, read it so it doesn't just sit in the buffer and note an error
    // Because $DATA shouldn't return anything, so if 
    Serial.print(Serial1.read());
    error = 1;
  }
  if(error){
    // try one more time
    Serial1.print("$DATA,");
    Serial1.print(String(baud));
    Serial1.write(';');
    delay(100);
    if(! Serial1.available()){
      error = 0;
      Serial.println("Maybe lowered baud rate?");
    }
    while(Serial1.available()){
      Serial.print(Serial1.read());
    }
  }
  if(error){
    Serial2.println("Could not lower baud rate");
    return 0;
  }
  Serial1.begin(baud);
  Serial1.setTimeout(SERIAL_TIMEOUT);
  Serial.println("Lowering arduino baud rate");
  // try n times, in case there's an error
  // possible that this will eventually get built into checkLKMcomm
  int nAttempts = 50;
  for(int i = 0; i < nAttempts; i++){
    if(checkLKMcomm(2)){
      Serial.println("Probably lowered baud rate, communicating with LKM");
      return 1;
    }
  }
  Serial.println("Maybe lowered baud rate, returning 0");
  return 0;
  // If failure, should probably communicate with ground too?
}


float parseLKM(){
  Serial.println("Called parseLKM");
  delay(100);
  String cmd, val;
  //read back in format #cmd,val;
  if(Serial1.available()) {
//    Serial1.readStringUntil('\n');
    cmd = Serial1.readStringUntil(delim);
    val = Serial1.readStringUntil(terminator);
  }
  Serial1.flush();
  // go through each command value pair and return the value or error
  return processCmdVal(cmd, val, false);
}



float processCmdVal(String cmd, String val, boolean save) {
  while(cmd.charAt(0)=='\n' || cmd.charAt(0)=='\r' ) {
    cmd = cmd.substring(1);
  }
  Serial.println(cmd);
  Serial.println(val);
  // go through each command value pair and return the value or error
  if (cmd.equals("#PWR") ){
    if(val.equals("ON")){
      val = "1.0";
    }
    else if (val.equals("OFF")) {
      val = "0.0";
    }
    else {
      return BAD_VAL;
    }
    return processTelem(val, PWR_INDEX, save);
  }
  else if (cmd.equals("#PULS") || cmd.equals("PULS") ){
    return processTelem(val, PULS_INDEX, save);
  }
  else if (cmd.equals("#DATA") || cmd.equals("DATA") ){
    return processTelem(val, DATA_INDEX, save);
  }
  else if (cmd.equals("#VOLT") || cmd.equals("VOLT") ){
    return processTelem(val, VOLT_INDEX, save);
  }
  else if (cmd.equals("#PRES") || cmd.equals("PRES") ){
    return processTelem(val, PRES_INDEX, save);
    Serial.print("cmd equals #PRES");
  }
  else if (cmd.equals("#TEMP") || cmd.equals("TEMP") ){
    return processTelem(val, TEMP_INDEX, save);
  }
  else if(cmd.equals("#MOTR") || cmd.equals("MOTR") ){
    return processTelem(val, MOTR_INDEX, save);
  }
  else {
    return BAD_CMD;
  }
}



float processTelem(String val, int index, bool saveStat) {
  float output;
  output = isValidTelem(val);
  if (saveStat) {
    stats[index] = output;
  }
  if (output == BAD_VAL) {
    return BAD_VAL;
  }
  else if (output == expected_val[index]){
    return output;
  }
  else if (index == VOLT_INDEX || index == PRES_INDEX || index == TEMP_INDEX) {
    expected_val[index] = output; // update expected_val
    return output;
  }
  else {
    return MISMATCH;
  }
}


float isValidTelem(String str){
  boolean isNum=false;
  for(byte i=0;i<str.length();i++)
  {
    if (i == 0) {
      isNum = isDigit(str.charAt(i)) || str.charAt(i) == '.' || str.charAt(i) == '-';
    }
    else {
      isNum = isDigit(str.charAt(i)) || str.charAt(i) == '.' ;
    }
    if(!isNum) {
      return BAD_VAL;
    }
  }
  return str.toFloat();
}



bool handleGroundCommand(){
  if(!Serial2.available()){
    return false;
  }
  Serial1.flush();
  lastGroundComm = millis();
  String command = Serial2.readStringUntil(delim);
  String arg = Serial2.readStringUntil(terminator);
  Serial2.flush();
  bool sendToLKM = 0;
  int dataIndex;
  Serial2.println("Recieved command " + command);
  if(command.equals("$PWR")){
      sendToLKM = 1;
      dataIndex = PWR_INDEX;
  }
  else if(command.equals("$PULS")){
      sendToLKM = 1;
      dataIndex = PULS_INDEX;
      expected_val[PULS_INDEX] = arg.toFloat();
  }
  else if(command.equals("$DATA")){
      sendToLKM = 1;
      dataIndex = DATA_INDEX;
      expected_val[DATA_INDEX] = arg.toFloat();
  }
  else if(command.equals("$VOLT")){
      Serial1.print("$VOLT;");
      dataIndex = VOLT_INDEX;
  }
  else if(command.equals("$PRES")){
      Serial1.print("$PRES;");
      dataIndex = PRES_INDEX;
  }
  else if(command.equals("$STAT")){
      Serial.print("STAT heard from ground");
      Serial1.print("$STAT;");
      delay(STAT_BUFFER_TIME);
  }
  else if(command.equals("$MOTR")){
      sendToLKM = 1;
      dataIndex = MOTR_INDEX;
      expected_val[MOTR_INDEX] = arg.toFloat();
  }
  else if(command.equals("$KP")){
    // how to do error checking here?
    // TODO
      PID_kP = arg.toFloat();
      Serial2.print("Changed KP to ");
      Serial2.print(PID_kP);
   }
   else if(command.equals("$KI")){
      PID_kI = arg.toFloat();
      Serial2.print("Changed KI to ");
      Serial2.print(PID_kI);
   }
   else if(command.equals("$KD")){
      PID_kD = arg.toFloat();
      Serial2.print("Changed KD to ");
      Serial2.print(PID_kD);
   }
   else if(command.equals("$EMERG_KILL1")){
      emergencyKillFirstSignal = true;
      firstKillSignalTime = millis();
      Serial2.print("First emergency kill signal recieved. Please send $EMERG_KILL2; to continue or $END_KILL; to abort.");
   }
   else if(command.equals("$EMERG_KILL2")){
    if(emergencyKillFirstSignal && (millis() - firstKillSignalTime < killSignalInterval)){
      Serial2.print("Emergency kill mode activated");
      emergencyKill = true;
    }
   }
   else if(command.equals("$END_KILL")){
    emergencyKill = false;
    emergencyKillFirstSignal = false;
    Serial2.print("Emergency kill sequence ended");
   }
   else if(command.equals("$LKM_POWERON")){
    // powers LKM on and adjusts the baud rates
    Serial2.print("Attempting LKM_POWERON, baud rate ");
    Serial2.print(arg);
    Serial2.print(powerLKMon(arg.toFloat()));
   }
   else if(command.equals("$ARDUINO_BAUD")){
    // changes payload arduino baud rate. Shouldn't be necessary to use with LKM_POWERON
    Serial1.begin(arg.toFloat());
    Serial1.setTimeout(SERIAL_TIMEOUT);
   }
   else if(command.equals("$SYSTEM_BAUD")){
    Serial2.print("Attempting to lowerBaudRate");
    Serial2.print(lowerBaudRate(arg.toFloat()));
   }
   else{
      // Complain to ground
      Serial2.write("Whatcha say?;");
      Serial2.flush();
   }
  
  if(sendToLKM){
    // Send command
    Serial1.write("$");
    Serial1.print(command);
    // Add the comma back
    Serial1.write(delim);
    // Send all the args
    Serial1.print(arg);
    Serial1.write(terminator);
  }
  delay(100);
  while(Serial1.available()){
    Serial2.write(Serial1.read());
  }
  Serial2.write('\n');
  recordVitals("Ground command: " + command + delim + arg + terminator);
  return true;
}


float parseStat(){
//  Serial.println("called parseStat()");
  delay(100);
  String stat;
  //read back stat;
  if(Serial1.available()) {
    stat = Serial1.readStringUntil(terminator);
//    Serial.println(stat);
  }
  Serial1.flush();
  
  int start = 0;
  int nextIndex = 0;
  String cmd, val;
  float result;
  boolean stat_error = false;
  
  // go through stat and process the cmd,val pairs
  while (nextIndex != -1) {
    nextIndex = stat.indexOf(',', start);
//    Serial2.print("next val start: ");
//    Serial2.println(nextIndex);
    if (nextIndex != -1) {
      cmd = stat.substring(start, nextIndex);
//      Serial2.print("cmd: ");
//      Serial2.println(cmd);
      start = nextIndex + 1;

      nextIndex = stat.indexOf(',', start);
//      Serial2.print("next cmd start: ");
//      Serial2.println(nextIndex);
      if (nextIndex != -1) {
        val = stat.substring(start, nextIndex);
//        Serial2.print("val: ");
//        Serial2.println(val);
      }
      else{
         val = stat.substring(start);
//         Serial2.print("val: ");
//         Serial2.println(val);
      }
      
    }
    else {
      stat_error = true;
    }
    start = nextIndex + 1;

    result = processCmdVal(cmd, val, true);
    if (result == BAD_CMD || result == BAD_VAL) {
      stat_error = true;
    }
    
  }// end while
  
  if (stat_error) {
    return BAD_STAT;
  }
  return 1.0;
}


float demandVal(String command, int nTrials){
  // Only works for function where parseLKM returns a value
  // Send command with semicolon attached
  for(int i = 0; i < nTrials; i++){
     Serial1.flush();
     Serial1.print(command);
     delay(100);
     float val = parseLKM();
     if(val != BAD_VAL && val != BAD_CMD){
        return val;
     }
  }
  Serial.println("demandVal failed, big yikers");
  return -1;
}


bool powerLKMon(int baudRate){
  if(!(baudRate == 9600 ||baudRate == 4800 || baudRate == 9600)){
    Serial2.println("Invalid baud rate, try again");
    return 0;
  }
  // Assumes the LKM is not on
  Serial1.print("$PWR,ON;");
  Serial1.flush();
  Serial1.begin(LKM_DEFAULT_BAUD);
  Serial1.setTimeout(SERIAL_TIMEOUT);
  delay(LKM_STARTUP_TIME);
  Serial2.println(Serial1.readStringUntil('\n'));
  Serial1.flush();
  return lowerBaudRate(baudRate);
}

bool monitorVals(){
  // Ask for stats
  Serial1.flush();
  Serial1.print("$STAT;");
  int timeout = 100;
  int timeoutCounter = 0;
  bool somethingChanged = false;
  while(parseStat() != 1){
    // If there's an error, keep going until there's not an error or we decide it got stuck
    timeoutCounter++;
    if(timeoutCounter > timeout){
      Serial2.println("Error with monitorVals, could not parseStat()");
      return 0;
    }
    Serial1.flush();
    Serial1.print("$STAT;");
  }
  if(stats[MOTR_INDEX] != expected_val[MOTR_INDEX]){
    float initialMotrVal = stats[MOTR_INDEX];
    somethingChanged = true;
    Serial1.print("$MOTR,");
    Serial1.print(expected_val[MOTR_INDEX]);
    Serial1.print(";");
    String errorMsg = "Automated LKM monitor: Changed MOTR from " + String(initialMotrVal) + " to " + String(expected_val[MOTR_INDEX]);
    Serial2.println(errorMsg);
    recordVitals(errorMsg);
  }
  if(stats[PULS_INDEX] != expected_val[PULS_INDEX]){
    float initialPulsVal = stats[PULS_INDEX];
    somethingChanged = true;
    Serial1.print("$PULS,");
    Serial1.print(expected_val[PULS_INDEX]);
    Serial1.print(";");
    String errorMsg = "Automated LKM monitor: Changed PULS from " + String(initialPulsVal) + " to " + String(expected_val[PULS_INDEX]);
    Serial2.println(errorMsg);
    recordVitals(errorMsg);
  }
  return somethingChanged;
}

double findCurrent(int batPin, int currentPin) {
  // TBH totally copied and pasted this from last term and I have no idea what it does, but maybe it'll be useful...?
  double currReading = analogRead(currentPin);
  double Vin = (3.3 / 1023.0) * currReading;
  double batReading = analogRead(batPin);
  double Vbat = (3.3 / 1023.0) * batReading;
  double divV = (4.7+ 22.0)/4.7;
  double R = 2.0;
  double current = (Vbat - Vin) * divV/R;
  return current * 1000; // in mA
}
