## Ground Control

This ground command module is responsible for sending commands and listening to an Arduino Due managing a payload (the LKM) on a HAB (High Altitude Balloon) flight. Communication is done through a serial link to an Arduino Uno connected as a serial passthrough to an xTend vB RF module. This command suite relies on a command line interface to send commands of three categories:

1. payload commands (directly relayed to the LKM)
2. box commands (meant for Arduino Due, for managing the LKM and other payload tasks)
3. adminstrative commands (for managing the state of the ground command module)

The script first lists available serial ports and prompts for the correct one the radio is connected to. Then it enters a "Listening Mode" that continuously polls the radio's serial buffer. If the user presses 'c', "Command Mode" is entered where the user may input commands that are sent over radio. Further detailed instructions are provided in the interface.

Note that while in "Command Mode", the script does not read from the buffer (due to blocking input) and there may be a backlog of messages upon exiting "Command Mode". 

### Usage
```
$ py ground.py -h
usage: ground.py [-h] [-l [LOG_FILENAME]] [-R [BAUD_RATE]] [-p [SERIAL_PORT]]
                 [-D] [-L [LOGGING_LEVEL]]

optional arguments:
  -h, --help          show this help message and exit
  -l [LOG_FILENAME]   set custom log filename. default: m-d-Y I-M-S p.log
  -R [BAUD_RATE]      set custom serial baud rate. default: 9600
  -p [SERIAL_PORT]    set serial port name. will prompt in interface if not
                      specified
  -D                  enable debug mode, suppresses file logging
  -L [LOGGING_LEVEL]  set logging level. default: DEBUG
```

### Python
This tool was developed in Python 3.7.2, and should be run in a virtualenv with the same version of Python interpreter. Install package requirements listed in `requirements.txt` with pip.

### virtualenv
1. Obtain virtualenv with
```
$ pip install virtualenv
```
2. Test the installation:
```
$ virtualenv --version
```
3. Create virtual environment in project directory:
```
$ virtualenv venv
```
4. Activate virtual environment:
```
$ source venv/Scripts/activate
```
5. install requirements
```
$ pip install -r requirements.txt
```
6. To deactivate:
```
$ deactivate
```

