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
#define RF_BAUD 2400
#define USB_BAUD 9600
#define CHAR_0 48
#define CHAR_9 57
#define REPEAT_RATE 5

void setup()
{
    // initialize communication with Computer
    Serial.begin(USB_BAUD);
    // initialize communication with RF modules 
    Serial1.begin(RF_BAUD);
    // initialize counter that we are sending over RF
    counter = 0;
}

void loop()
{
    // send number first, only send up to counter = 1000
    if (counter < 1000)
    {
        Serial1.print(repeat(counter++));
    }
    // read number back
    if (Serial1.available() > 0)
    {
        char c;
        while (Serial1.available() > 0)
        {
            c = Serial1.read();
            // if reached end of transmission, next line
            if (c == ',')
            {
                Serial.println();
            }
            // otherwise still receiving data from transmission, so print
            else
            {
                // the below isDigit() check is our funky way of filtering out
                // lots of noise (in between?) transmissions. Try printing c 
                // without the isDigit() check to get a feel of how much garbage
                // there is. - Evan
                if (isDigit(c))
                {
                    Serial.print(c);
                }
            }
        }
    }

    // wait a second before next round to avoid crowding the serial monitor
    delay(1000);
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
