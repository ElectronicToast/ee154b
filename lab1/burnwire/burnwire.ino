#define BURN_WIRE_PIN 3
#define DUTY_MIN 0
#define DUTY_MAX 255

void setup() {
  // initialize serial:
  Serial.begin(9600);
  pinMode(BURN_WIRE_PIN, OUTPUT);
}

String str = "";

void loop() {
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (isDigit(ch)) {
      str += ch;
    }
    if (ch == '\n') {
      if (!str.length()) {
        Serial.println("Error: no valid digits sent");
      } else {
        int duty = str.toInt();
        if (duty > DUTY_MAX || duty < DUTY_MIN) {
          Serial.print("Error: duty must be between ");
          Serial.print(DUTY_MIN);
          Serial.print(" and ");
          Serial.println(DUTY_MAX);
        } else {
          analogWrite(BURN_WIRE_PIN, duty);
          Serial.print("Burn wire duty: ");
          Serial.println(duty);
        }
        str = "";
      }
    }
  }
}
