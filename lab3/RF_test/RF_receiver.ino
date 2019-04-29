/*
  Simple Receiver Code
  (TX out of Arduino is Digital Pin 1)
  (RX into Arduino is Digital Pin 0)
*/
int incomingByte = 0;
int i;
int majoritynum;
int errors = 0;
int nums[3];
void setup() {
  //2400 baud for the 434 model
  Serial.begin(2400);
}
void loop() {
  // read in values, debug to computer
  // int j = 1;          // j is our expected value.
  // while (j <= 1000) { //we are assuming that we will be sent numbers 1 through 1000
  if(Serial.available()>0){
      for (i = 0; i < 3; i++) {
    while (Serial.available() > 0) {

      int x = 0;
      char checkbyte = Serial.read();
      if (checkbyte != ',') {
        int digits = checkbyte - '0';
        x = x * 10 + digits;
      }
      //Serial.println(x);
      nums[i] = x;
    }
  }
    majoritynum = floor((nums[0] + nums[1] + nums[2]) / 2);
    Serial.println(majoritynum);


  //      j = j + 1;
       if (1 != majoritynum) {
        errors = errors + 1;
 // Serial.println(errors);
  }

  }

}
//}
