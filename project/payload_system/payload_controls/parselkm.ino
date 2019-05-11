// reads the stuff from the LKM and return the value read
//
// if we get back a bad command, we'll return -1 instead
// if we get back a bad value, we'll return -2 instead
// if we get back something mismatching the expected_val for 
//     PWR, PULS, DATA, MOTR, we'll return -3 instead
// if we get back something mismatching the expected_val for 
//     VOLT, PRES, TEMP, we'll just update expected_val
//
// note: for PWR, ON will be encoded as 1.0, OFF as 0.0

#define BAD_CMD -1
#define BAD_VAL -2
#define MISMATCH -3

#define PWR_INDEX  0
#define PULS_INDEX 1
#define DATA_INDEX 2
#define VOLT_INDEX 3
#define PRES_INDEX 4
#define TEMP_INDEX 5
#define MOTR_INDEX 6

// global arrays to store telem 
// [PWR, PULS, DATA, VOLT, PRES, TEMP, MOTR]
float expected_val[] = {1.0, 0.0, 9600, 5.0, 1000.0, 27.0, 30.0};

// define how to parse our telem
char delim = ',';
char terminator = '\n';

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    Serial.println(parseLKM());
  }
}


float parseLKM(){
  delay(100);
  String cmd, val;
  //read back in format #cmd,val;
  if(Serial.available()) {
    cmd = Serial.readStringUntil(delim);
    val = Serial.readStringUntil(terminator);
  }

  // go through each command value pair and return the value or error
  if (cmd.equals("#PWR")){
    if(val.equals("ON")){
      val = "1.0";
    }
    else if (val.equals("OFF")) {
      val = "0.0";
    }
    else {
      return BAD_VAL;
    }
    return processTelem(val, PWR_INDEX);
  }
  else if (cmd.equals("#PULS")){
    return processTelem(val, PULS_INDEX);
  }
  else if (cmd.equals("#DATA")){
    return processTelem(val, DATA_INDEX);
  }
  else if (cmd.equals("#VOLT")){
    return processTelem(val, VOLT_INDEX);
  }
  else if (cmd.equals("#PRES")){
    return processTelem(val, PRES_INDEX);
  }
  else if (cmd.equals("#TEMP")){
    return processTelem(val, TEMP_INDEX);
  }
  else if(cmd.equals("#MOTR")){
    return processTelem(val, MOTR_INDEX);
  }
  else {
    return BAD_CMD;
  }
}

float processTelem(String val, int index) {
  float output;
  output = isValidTelem(val);
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
      return -2.0;
    }
  }
  return str.toFloat();
}
