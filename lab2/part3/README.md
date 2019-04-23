### Usage

Run in virtual env with instructions in README in lab2/.

```
$ py part3/commander.py -h
usage: commander.py [-h] [-l [LOG_FILENAME]] [-R [BAUD_RATE]]
                    [-p [SERIAL_PORT]] [-D] [-L [LOGGING_LEVEL]]

optional arguments:
  -h, --help          show this help message and exit
  -l [LOG_FILENAME]   set custom log filename. default: m-d-Y I-M-S p.log
  -R [BAUD_RATE]      set custom serial baud rate. default: 9600
  -p [SERIAL_PORT]    set serial port name. default: COM3
  -D                  enable debug mode, suppresses file logging
  -L [LOGGING_LEVEL]  set logging level. default: DEBUG
```

### Example 
```
$ py part3/commander.py -D -p "COM10"
Serial port COM10 opened successfully.
--------------------------------------------------------
        0: Increment a payload string by one character.
        1: Decrement a payload string by one character.
        2: Read a thermistor and transmit room temperature in Celsius.
        3: Read a thermistor and transmit room temperature in Fahrenheit.
        4: Reset the Arduino and return the number of resets stored by the Arduino EEPROM.
        5: Begin recording time-tagged thermistor temperature to EEPROM every 30 seconds.
        6: Stop recording thermistor temperature to EEPROM.
        7: Readback times and temperatures from EEPROM memory.
        8: Readback every other time / temperature from EEPROM memory.
--------------------------------------------------------
>>0
Command: 0 | Increment a payload string by one character.
Payload string >> abcdef
Sending... 0abcdef
Recieved... bcdefg
```