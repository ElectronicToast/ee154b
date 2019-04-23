### Usage

Run in virtual env with instructions in README in lab2/.

```
$ py loopback_test.py -h
usage: loopback_test.py [-h] [-l [LOG_FILENAME]] [-R [BAUD_RATE]] [-e]
                        [-p [SERIAL_PORT]]

optional arguments:
  -h, --help         show this help message and exit
  -l [LOG_FILENAME]  log filename
  -R [BAUD_RATE]     serial baud rate
  -e                 encode or not
  -p [SERIAL_PORT]   set serial port name. default: COM3
```

### Example 
To run a test at a baud rate of 9600 with encoding, on port COM10:
```
$ py loopback_test.py -l '9600.log' -R 9600 -e -p 'COM10'
```