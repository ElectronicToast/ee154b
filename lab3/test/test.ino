void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

float computePWM(int pin, int nTrials){
  // PWM period, in ms, from pg 47 of the XTend manual
  float pwmPeriod = 8320;
  // Averages high-pulse lengths over nTrials
  float pulseSum = 0;
  for(int i = 0; i < nTrials; n++){
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
  // 
  on the XTend, into any digital pin on the Arduino
  float pwm = computePWM(pin, nTrials);
  return dBAboveSensitivity(pwm);
}
