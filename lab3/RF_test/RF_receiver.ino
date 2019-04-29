/*
  Simple Receiver Code
  (TX out of Arduino is Digital Pin 1)
  (RX into Arduino is Digital Pin 0)
*/
int incomingByte = 0;
int i;
int expectedval = 0;
int majoritynum;
int errors = 0;
int nums[3];
void setup() {
  //2400 baud for the 434 model
  Serial.begin(2400);
}
void loop() {
  i = 0;
  if (Serial.available()>0 && Serial.available()%3 == 0 && expectedval<=1000) {
    
    expectedval = expectedval+1;
    //Serial.println(expectedval);
    while (Serial.available() > 0) {
      int x = 0;
      char checkbyte = Serial.read();
      if (checkbyte != ',') {
        int digits = checkbyte - '0';
        x = x * 10 + digits;
      }
      Serial.println(x);
      Serial.println();
      nums[i] = x;
      i = i + 1;
    }
    if (Serial.available() <= 0) {
      if(nums[0] == nums[1] || nums[0] == nums[2]){
        majoritynum = nums[0];
      }
      else if(nums[1] == nums[2]){
        majoritynum = nums[1];
      }
      else{
        majoritynum = 0; 
      }
      if (expectedval != majoritynum) {
        errors = errors + 1;
        //Serial.println(errors);
      }
    }
}

}
