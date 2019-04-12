/*
 * For Part 2 of the lab, controlling temperature at end of aluminum block.
 * For 2a @ 30C, kp = 600, ki = kd = 0
 * For 2b @ 3C, kp = 600, ki = 0, kd = 200
 */

//PID constants
double kp = 600;
double ki = 0;
double kd = 200;
// desired temperature in celsius
int setPoint = 3;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double cumError, rateError;

// Logarithmic Thermistor calibration
#define TMST_LOG_SLOPE -22.025
#define TMST_LOG_INT 228.406

// Thermistor pins
const int TMST_1_PIN = 0;
const int TMST_2_PIN = 1;
const int TMST_3_PIN = 2;

// ADC Resolution
const double ADC_RES = 1024.0;

// Heat pad pin
const int HEAT_PAD_PIN = 3;

void setup() {
  Serial.begin(9600);
  Serial.println("Program started...");

  pinMode(HEAT_PAD_PIN, OUTPUT);
  pinMode(TMST_1_PIN, INPUT);
  pinMode(TMST_2_PIN, INPUT);
  pinMode(TMST_3_PIN, INPUT);
}

void loop(){
  double tmst_1_raw = (double) analogRead(TMST_1_PIN);
  double r = 10000.0 * tmst_1_raw / (ADC_RES - tmst_1_raw);
  double input = TMST_LOG_SLOPE * log(r) + TMST_LOG_INT;

  int out = computePID(input);
  if(out > 255) {
    out = 255;
  } else if(out < 0) {
    out = 0;
  }

  //Serial.print("input: ");
  Serial.print(input);
  //Serial.print("\terror: ");
  Serial.print(",");
  Serial.print(error); 
  //Serial.print("\toutput: ");
  Serial.print(",");
  Serial.print(out);
  Serial.println();
  
  delay(100);
  analogWrite(HEAT_PAD_PIN, out);
}

double computePID(double inp){     
  currentTime = millis();                               //get current time
  elapsedTime = (double)(currentTime - previousTime);   //compute time elapsed from previous computation
  
  error = setPoint - inp;                               // determine error
  cumError += error * elapsedTime;                      // compute integral
  rateError = (error - lastError)/elapsedTime;          // compute derivative

  double out = kp*error + ki*cumError + kd*rateError;   //PID output               

  lastError = error;                                    //remember current error
  previousTime = currentTime;                           //remember current time
  
  return out;                                           //have function return the PID output
}
