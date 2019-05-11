// reads the stat from the LKM and returns 1.0 when successful stat
//                                 returns -4.0 when error in the stat
// errors could be bad format, bad command, bad val
// mismatch does not count as an error
//
// -2 will be saved in stat for where there is a bad val 
// 
// note: for PWR, ON will be encoded as 1.0, OFF as 0.0

/////////////////// some TESTS SHOWN BELOW ///////////////////////
// first line is what is received                               //
// second is return from parseStat                              //
// third is what is saved in stat array                         //
//////////////////////////////////////////////////////////////////
//#PWR,ON,PULS,0,DATA,9600,VOLT,5.0,PRES,1000,TEMP,27.0,MOTR,30 //
//1.00                                                          //
//1.00, 0.00, 9600.00, 5.00, 1000.00, 27.00, 30.00,             //
//                                                              //
//#PWR,ON,PULS,0,DATA,2400,VOLT,5.0,PRES,1000,TEMP,27.0,MOTR,30 //
//1.00                                                          //
//1.00, 0.00, 2400.00, 5.00, 1000.00, 27.00, 30.00,             //
//                                                              //
//#PWR,ON,PULS,0,DATA,9600,VOLT,5.0,PRES,1000,TEMP27.0,MOTR,30  //
//-4.00                                                         //
//1.00, 0.00, 9600.00, 5.00, 1000.00, 27.00, 30.00,             //
//                                                              //
//#PWR,ON,PULS,0,DATA,9600,VOLT,5.0,PRES,10,00,TEMP,27.0,MOTR,30//
//-4.00                                                         //
//1.00, 0.00, 9600.00, 5.00, 10.00, 27.00, 30.00,               //
//                                                              //
//#PWR,ON,PULS,0,DATA,9600,VOLT,5.0,PRES,1000,TEMP,27.0,MOTR,30,//
//-4.00                                                         //
//1.00, 0.00, 9600.00, 5.00, 1000.00, 27.00, 30.00,             //
//                                                              //
//#PWR,ON,PLS,0,DATA,9600,VOLT,5.0,PRES,1000,TEMP,27.0,MOTR,30  //
//-4.00                                                         //
//1.00, 0.00, 9600.00, 5.00, 1000.00, 27.00, 30.00,             //
//                                                              //
//#PWR,ON,PULS,0,DATA,96f0,VOLT,5.0,PRES,1000,TEMP,27.0,MOTR,30 //
//-4.00                                                         //
//1.00, 0.00, -2.00, 5.00, 1000.00, 27.00, 30.00,               //
//////////////////////////////////////////////////////////////////



// error codes
#define BAD_CMD -1
#define BAD_VAL -2
#define MISMATCH -3
#define BAD_STAT -4

// indices of where values are stored in the global arrays expected_val and stat
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
float stat[7] ;


// define how to parse our telem ###CHANGE terminator to ';'###
char delim = ',';
char terminator = '\n';

void setup() {
  Serial.begin(2400);
  Serial.setTimeout(2000); // because reading the full stat string takes a long time
}

void loop() {
  // to test parseStat():
  if(Serial.available()){
    Serial.println(parseStat());
    for (int i = 0; i < 7; i++) {
      Serial.print(stat[i]);
      Serial.print(", ");
    }
    Serial.println('\n');
  }

//  // to test parseLKM():
//  if(Serial.available()){
//    Serial.println(parseLKM());
//  }
}


float parseStat(){
  delay(100);
  String stat;
  //read back stat;
  if(Serial.available()) {
    stat = Serial.readStringUntil(terminator);
    Serial.println(stat);
  }
  
  int start = 0;
  int nextIndex = 0;
  String cmd, val;
  float result;
  boolean stat_error = false;
  
  // go through stat and process the cmd,val pairs
  while (nextIndex != -1) {
    nextIndex = stat.indexOf(',', start);
//    Serial.print("next val start: ");
//    Serial.println(nextIndex);
    if (nextIndex != -1) {
      cmd = stat.substring(start, nextIndex);
//      Serial.print("cmd: ");
//      Serial.println(cmd);
      start = nextIndex + 1;

      nextIndex = stat.indexOf(',', start);
//      Serial.print("next cmd start: ");
//      Serial.println(nextIndex);
      if (nextIndex != -1) {
        val = stat.substring(start, nextIndex);
//        Serial.print("val: ");
//        Serial.println(val);
      }
      else{
         val = stat.substring(start);
//         Serial.print("val: ");
//         Serial.println(val);
      }
      
    }
    else {
      stat_error = true;
    }
    start = nextIndex + 1;

    result = processCmdVal(cmd, val, true);
    if (result == -1 || result == -2) {
      stat_error = true;
    }
    
  }// end while
  
  if (stat_error) {
    return BAD_STAT;
  }
  return 1.0;
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
  return processCmdVal(cmd, val, false);
}



float processCmdVal(String cmd, String val, boolean save) {

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
    stat[index] = output;
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
      return -2.0;
    }
  }
  return str.toFloat();
}
