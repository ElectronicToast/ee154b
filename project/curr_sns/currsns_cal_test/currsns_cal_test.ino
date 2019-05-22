/**
 * currsns_cal_test.ino - test sketch for verifying current sensor calibration 
 *                        curve
 *
 * Hardware:
 *      Connect sensor output to analog pin with a 22k:10k resistive divider 
 *      to divide down 5V to 3V3
 *
 *      CONNECT SENSOR VCC to 5V!!!
 *
 *      Compatible with both 5V and 3V3 Arduinos
 *
 * Revision History:
 *      05/20/2019      Ray Sun         This isn't a G.G. class
 */

//                              Flapjack values
#define         CURR_OFF        1990    // Old values: 658.8
#define         CURR_GAIN       -1.6    // -0.4
const int analogInPin = A0;

// Number of samples to average the reading over
// Change this to make the reading smoother... but beware of buffer overflows!
const int avgSamples = 10;

int sensorValue = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // read the analog in value:
  for (int i = 0; i < avgSamples; i++)
  {
    sensorValue += analogRead(analogInPin);

    // wait 2 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(2);
  }

  sensorValue = sensorValue / avgSamples;

  // The on-board ADC is 10-bits -> 2^10 = 1024 -> 5V / 1024 ~= 4.88mV
  // The voltage is in millivolts
  // Also apply calibration curve
  // current = (4.88 * sensorValue - CURR_OFF) / CURR_GAIN
  float current = (4.88 * sensorValue - CURR_OFF) / CURR_GAIN;

  Serial.print(current);

  // -- DO NOT UNCOMMENT BELOW THIS LINE --
  Serial.print("\n");

  // Reset the sensor value for the next reading
  sensorValue = 0;
}
