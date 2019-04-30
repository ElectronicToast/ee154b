/*
This sketch transmits and recieves numbers (1-1000) over RF using
Laipac TLP/RLP 434 modules.

The RF modules are connected through Serial1 on the Arduino Due,
while Serial (USB Serial) is used to see what is recieved over RF. 

Serial TX: Computer USB
Serial RX: Computer USB

Serial1 TX: TLP 434
Serial1 RX: RLP 434
*/

int counter;
int errors;
#define RF_BAUD 2400
#define USB_BAUD 9600
#define CHAR_0 48
#define CHAR_9 57
#define REPEAT_RATE 20

void setup()
{
    // initialize communication with Computer
    Serial.begin(USB_BAUD);
    // initialize communication with RF modules 
    Serial1.begin(RF_BAUD);
    // initialize counter that we are sending over RF
    // we know transmission under 100 are going to be garbage with our
    // decoding scheme, so start at 100
    counter = 1;
    errors = 0;
}

void loop()
{
    // send number first, only send up to counter = 1000
    if (counter < 1001)
    {
        Serial1.print(counter);
    }
    else
    {
        Serial.print("Error rate: ");
        Serial.print((double) errors / 10);
        Serial.print("%");
        while(1){}
    }
    // read number back
    if (Serial1.available() > 0)
    {
        char c;
        String encoded = "";
        while (Serial1.available() > 0)
        {
            c = Serial1.read();
            if (isDigit(c))
            {
                encoded += c;
            }
        }
        // reach end of transmission
        {
            int decoded = encoded.toInt();
        
            if (decoded != counter-1)
            {
                errors++;
            }
            Serial.print(counter-1);
            Serial.print(" ---> ");
            Serial.print(decoded);
            Serial.print("\t Error rate (%): ");
            Serial.print((double) errors / 10);
            Serial.println();
        }
    }

    // wait a second before next round to avoid crowding the serial monitor
    delay(300);
    counter++;
}

// returns an input repeated REPEAT_RATE times, ended with a comma
String repeat(int in)
{
    String out = "";
    for (int i = 0; i < REPEAT_RATE; i++)
    {
        out += String(in);
    }
    return out + ",";
}

// checks if a character is a digit
bool isDigit(char c)
{
    if ((c >= CHAR_0) && (c <= CHAR_9))
        return true;
    return false;
}

// decodes the message by taking the last 3 digits and somehow offsetting
// by 2
int decode(String encoded)
{
    if (encoded.length() >= 3)
    {
        return encoded.substring(encoded.length() - 3).toInt() + 2;
    }
    return 0;
}
