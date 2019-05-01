
/*
 * Lab3_pt2_v2.ino - Sketch for interfacing a XTEND module with a laptop serial 
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
 * Establishes communication between the client and the XTEND module. Client 
 * can set a parameter of the instrument using the format seen in case 'b', 
 * get a parameter using the format seen in case 'a', or return the RSSI 
 * to the client as seen in case 'c'. 
 * The transmit and receive algorithms are implementations of Lab 3 Part 1.
 */

#define RF_BAUD 2400
#define USB_BAUD 9600
#define CHAR_0 48
#define CHAR_9 57
#define REPEAT_RATE 20
#define STARTUP_DELAY 2000
#define RSSI_PIN 3

void setup() {
  // Initialize communication with Computer
  Serial.begin(USB_BAUD);
  // Initialize communication with RF Modules
  Serial1.begin(RF_BAUD);
  // Setup the RSSI Pin 
  pinMode(RSSI_PIN, INPUT);

  // Wait a bit 
  delay(STARTUP_DELAY);
}

void loop() {

  int input = '#';
  int delim = '\n';
  int numRepeats = 20;
  if(Serial.available()){
        input = Serial.read();
        Serial.write(delim);
    }
  
  switch(input) {
    // "Get"'s
    case 'a': {
      int receivedData = -2;
      // send command
      Serial1.print(repeat(input, numRepeats));
      delay(100);

      // receive telemetry and send to laptop
      while (receivedData < 0) {
        receivedData = rx(numRepeats);
        if (receivedData == -1) {
          Serial1.print(repeat(input, numRepeats));
        }
        else if(receivedData >= 0) {
          Serial.print(String(receivedData));
          Serial.print('\n');
        }
      }
      break;
    }

    //"Set"'s
    case 'b': {
      // send command
      Serial1.print(repeat(input, numRepeats));
      delay(100);
      Serial.print('\n');
     
      // laptop will then prompt for the argument that we send 
      int data = '\0';
      while(data != delim) {
        if (Serial.available() > 0) {
          data = Serial.read();
          if(data != delim) {
            Serial1.print(data);
          }
        }
      }
      Serial.write('\n');
      break;
    }

    // Get and send RSSI
    case 'c': {
      int nTrials = 10;
      Serial.print(String(computedBaboveSentitivity(RSSI_PIN, nTrials), 2));
      Serial.print(delim);
      break;
    } 
  }     
}

// returns an input repeated REPEAT_RATE times, ended with a comma
String repeat(char in, int numRepeats)
{
    String out = "";
    for (int i = 0; i < numRepeats; i++)
    {
        out += String(in);
    }
    return out + ",";
}

// int data should be transmitted numRepeats number of times and end with ','
// returns -1 if an error is detected
// returns -2 if no complete data received
int rx(int numRepeats)  {
  
  int i = 0;
  int majorityVal;
  int data[numRepeats];

  if (Serial1.available()>0 && Serial1.available()%numRepeats == 0) {
    
    while (Serial1.available() > 0) {
      int x = 0;
      char checkbyte = Serial1.read();
      if (checkbyte != ',') {
        int digits = checkbyte - '0';
        x = x * 10 + digits;
      }
      Serial.println(x);
      Serial.println();
      data[i] = x;
      i = i + 1;
    }
    // find the value that we've received more than half the time
    if (Serial1.available() <= 0) {
      majorityVal = data[0];
      int majorityCt = 0;
      int nextVal = majorityVal;
      int start = 0;
      while (majorityCt < (numRepeats/2 + 1)) {
        if (start > (numRepeats/2 + 1) ) {
          return -1;
        }
        for(int j = start; j< numRepeats; j++) {
          if (data[j] = majorityVal) {
            majorityCt ++;
          }
          else if (nextVal == majorityVal) {
            nextVal = data[j];
            start = j;
          }
        }

        if (majorityCt < (numRepeats/2 + 1) ) {
          majorityVal = nextVal;
          majorityCt = 0;
        }
      }

      return majorityVal;
    } 
    
  }
  return -2;
}


float computePWM(int pin, int nTrials){
  // PWM period, in us, from pg 47 of the XTend manual
  float pwmPeriod = 8320;
  // Averages high-pulse lengths over nTrials
  float pulseSum = 0;
  for(int i = 0; i < nTrials; i++){
    pulseSum += pulseIn(pin, HIGH, pwmPeriod);
  }
  return pulseSum / nTrials / pwmPeriod;
}

float dBAboveSensitivity(float pwm){
  // Assuming we can extrapolate from the chart on pg 47
  return -10 + .3/20.0 * pwm;
}

float computedBaboveSentitivity(int pin, int nTrials){
  // Runs the full thing do get dB above the radio sensitivity. Read from pin 11 
  // on the XTend, into any digital pin on the Arduino
  float pwm = computePWM(pin, nTrials);
  return dBAboveSensitivity(pwm);
}
