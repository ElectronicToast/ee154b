/*
This sketch transmits and recieves numbers (1-1000) over RF using
Laipac TLP/RLP 434 modules.

The RF modules are connected through Serial1 on the Arduino Due,
while Serial (USB Serial) is used to see what is recieved over RF. 

Serial TX: Computer USB
Serial RX: Computer USB

Serial1 TX: TLP 434
Serial1 RX: RLP 434

The encoding scheme is a repetition code with a '*' as a delimiter.
*/

int counter;
int errors;
#define RF_BAUD 2400
#define USB_BAUD 9600
#define CHAR_0 48
#define CHAR_9 57
#define REPEAT_RATE 3
#define DELIM '*'

void setup()
{
    // initialize communication with Computer
    Serial.begin(USB_BAUD);
    // initialize communication with RF modules 
    Serial1.begin(RF_BAUD);
    counter = 1;
    errors = 0;
}

void loop()
{
    // ******* TX *******
    // send number first, only send up to counter = 1000
    if (counter < 1000)
    {
        Serial1.print(encode(counter));
    }
    else
    {
        // print final error and hang if done sending 1000 numbers
        Serial.print("Error rate: ");
        Serial.print((double) errors / 10);
        Serial.print("%");
        while(1){}
    }

    // ******* RX *******
    if (Serial1.available() > 0)
    {
        char c;
        String encoded = "";
        while (Serial1.available() > 0)
        {
            c = Serial1.read();
            // filter out noise thats not a digit or the delimiter
            if (isDigit(c) || c == DELIM)
            {
                encoded += c;
            }
        }

        // reach end of message
    
        // decode message and validate
        int decoded = decode(encoded);
        // somehow there's a 1-off error -> dont have to worry
        if (decoded != counter - 1)
        {
           errors++;
        }
        
        // print results
        Serial.print(counter - 1);
        Serial.print(" ---> ");
        Serial.print(encoded);
        Serial.print(" ---> ");
        Serial.print(decoded);
        Serial.print("\t Error rate (%): ");
        Serial.print((double) errors / 10);
        Serial.println();
        
    }

    // wait before next round to avoid crowding
    delay(300);
    counter++;
}

// returns an input repeated REPEAT_RATE times, with DELIM as a delimiter
// e.g. 56 -> *56*56*56*
String encode(int in)
{
    String out = String(DELIM);
    for (int i = 0; i < REPEAT_RATE; i++)
    {
        out += String(in);
        out += DELIM;
    }
    return out;
}

// checks if a character is a digit
bool isDigit(char c)
{
    if ((c >= CHAR_0) && (c <= CHAR_9))
        return true;
    return false;
}

// decodes a repepition encoded string
int decode(String encoded)
{
    // see if there are DELIM's at all
    if (encoded.indexOf(DELIM) == -1 || encoded.indexOf(DELIM) == encoded.lastIndexOf(DELIM))
    {
        return -1;
    }

    // trim down so starts right after first DELIM and ends with last DELIM
    encoded = encoded.substring(encoded.indexOf(DELIM) + 1, encoded.lastIndexOf(DELIM) + 1);

    // keep track of values between DELIM's
    int vals[REPEAT_RATE];
    // keep track of counts of vals[REPEAT_RATE] that appear in message
    int counts[REPEAT_RATE];
    // where to insert next unique read-in number
    int nextEntry = 0;
    
    // loop until finish parsing 
    while (encoded.length() > 1)
    {
        // get current number between DELIM's
        int num = encoded.substring(0, encoded.indexOf(DELIM)).toInt();
        encoded = encoded.substring(encoded.indexOf(DELIM) + 1);
        // to ignore cases where DELIM's are back to back and num = 0
        if (num != 0)
        {
            // find if an existing entry matches current num
            bool found = false;
            for (int j = 0; j < nextEntry; j++)
            {
                if (num == vals[j])
                {
                    counts[j]++;
                    found = true;
                    break;
                }
            }
            // we have found a unique number, insert into vals
            if (!found)
            {
                vals[nextEntry++] = num;
            }
        }
    }

    // done parsing through numbers, pick highest occuring one

    int max_val = vals[0];
    int max_counts = counts[0];
    
    for (int j = 0; j < nextEntry; j++)
    {
        if (counts[j] > max_counts)
        {
            max_val = vals[j];
        }
    }
    
    return max_val;
}
