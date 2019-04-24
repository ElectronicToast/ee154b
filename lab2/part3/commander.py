'''
This command module allows commands and messages to be sent and recieved by
an Arduino Uno over UART serial. Usage of the program is as follows:

usage: commander.py [-h] [-l [LOG_FILENAME]] [-R [BAUD_RATE]]
                    [-p [SERIAL_PORT]] [-D] [-L [LOGGING_LEVEL]]

optional arguments:
  -h, --help          show this help message and exit
  -l [LOG_FILENAME]   set custom log filename. default: m-d-Y I-M-S p.log
  -R [BAUD_RATE]      set custom serial baud rate. default: 9600
  -p [SERIAL_PORT]    set serial port name. default: COM3
  -D                  enable debug mode, suppresses file logging
  -L [LOGGING_LEVEL]  set logging level. default: DEBUG

Possible commands are:
    0: Increment a payload string by one character.
    1: Decrement a payload string by one character.
    2: Read a thermistor and transmit room temperature in Celsius.
    3: Read a thermistor and transmit room temperature in Fahrenheit.
    4: Reset the Arduino and return the number of resets stored by the Arduino EEPROM.
    5: Begin recording time-tagged thermistor temperature to EEPROM every 30 seconds.
    6: Stop recording thermistor temperature to EEPROM.
    7: Readback times and temperatures from EEPROM memory.
    8: Readback every other time / temperature from EEPROM memory.
'''

from time import sleep
import serial, random, string, sys, logging, argparse
from datetime import datetime

# Constants
DEFAULT_BAUD_RATE = 9600
DEFAULT_SERIAL_PORT = 'COM3' # on windows
DEFAULT_LOGGING_LEVEL = 'DEBUG'
RECORD_TEMP_START = '$' # the character sent back from Arduino if successful
RECORD_TEMP_STOP = '%' # the character sent back from Arduino if successful
EOM_CHAR = '#' # marks end of time temperature data
TIME_TEMP_DELIM = ',' # delimits time from temperature
TIME_OUT = 5    # time to wait for a response in seconds

ser = None
logger = None

MENU_ITEMS = [
    'Increment a payload string by one character.',
    'Decrement a payload string by one character.',
    'Read a thermistor and transmit room temperature in Celsius.',
    'Read a thermistor and transmit room temperature in Fahrenheit.',
    'Reset the Arduino and return the number of resets stored by the Arduino EEPROM.',
    'Begin recording time-tagged thermistor temperature to EEPROM every 30 seconds.',
    'Stop recording thermistor temperature to EEPROM.',
    'Readback times and temperatures from EEPROM memory.',
    'Readback every other time / temperature from EEPROM memory.']

def main(args):
    config_logger(args)

    # Open the serial port
    global ser
    try:
        ser = serial.Serial(args['serial_port'], args['baud_rate'])
    except serial.serialutil.SerialException:
        logger.error('Serial port ' + args['serial_port'] + ' in use or incorrect.')
        logger.info('Program terminated.')
        sys.exit()

    # Wait 1 second before sending characters
    sleep(1)

    # Confirm that the connection is established
    if ser.isOpen():
        logger.info('Serial port ' + args['serial_port'] + ' opened successfully.')

    while True:
        print_menu()
        i = input('>> ')

        if i.lower() == 'q' or i.lower() == 'quit':
            logger.info('Program terminated.')
            break
        elif i.isdigit() and int(i) >= 0 and int(i) < len(MENU_ITEMS):
            logger.info('Command: ' + str(i) + ' | ' + MENU_ITEMS[int(i)])

            if i == '0' or i == '1':
                body = input('Payload string >> ')
                logger.info('Sending... ' + i + body)
                serial_send(i + body)
                logger.info('Received... ' + serial_read())
            elif i == '5':
                serial_send(i)
                if serial_read() == RECORD_TEMP_START:
                    logger.info('Temperature logging started.')
                else:
                    logger.info('Start temperature logging failed.')
            elif i == '6':
                serial_send(i)
                if serial_read() == RECORD_TEMP_STOP:
                    logger.info('Temperature logging stopped.')
                else:
                    logger.info('Stop temperature logging failed.')
            elif i == '7' or i == '8':
                serial_send(i)
                logger.info('Reading saved (time, thermistor) values...')
                count = temperature_read()
                logger.info('Done reading ' + str(count) + ' values.')
            else: # commands 2,3,4
                serial_send(i)
                logger.info('Received... ' + serial_read())
        else:
            logger.info('Command "' + i + '" not recognized.')

# encodes a message using utf-8 and sends over serial
def serial_send(msg):
    ser.write(bytes(msg, 'utf-8'))

# waits for data to come in buffer, then read all data
def serial_read():
    serial_wait()
    out = ''
    while ser.in_waiting > 0:
        out += ser.read(1).decode('utf-8')
        sleep(0.01)
    return out

# reads and logs sets of (time, temperature) data
def temperature_read():
    serial_wait()
    out = ''
    count = 0
    while ser.in_waiting > 0:
        serial_wait()
        c = ser.read(1).decode('utf-8')
        if c == TIME_TEMP_DELIM:
            count += 1
            if count % 2 == 1:
                out += ', '
            else:
                logger.info(out)
                out = ''
        elif c == EOM_CHAR:
            break
        else:
            out += c
        sleep(0.01)

    return int(count / 2)

# hangs until data comes into the buffer
def serial_wait():
    # wait for data to come in
    start = datetime.now().second
    while ser.in_waiting == 0:
        if datetime.now().second - start > TIME_OUT:
            logger.error('Read timeout!!!!')
            break
        pass

# prints all menu items (printed only to stdout, not to log file)
def print_menu():
    print('--------------------------------------------------------')
    for i in range(len(MENU_ITEMS)):
        print('\t' + str(i) + ': ' + MENU_ITEMS[i])
    print('\t' + '"Q": Quit program.')
    print('--------------------------------------------------------')

# parses command line arguments
def parse_arguments(argv):
    default_filename = datetime.now().strftime("%m-%d-%Y %I-%M-%S %p")+".log"

    parser = argparse.ArgumentParser()

    # filename handling
    parser.add_argument('-l',
        nargs='?',
        dest='log_filename',
        const=default_filename,
        default=default_filename,
        help='set custom log filename. default: m-d-Y I-M-S p.log')

    # baud rate handling
    parser.add_argument('-R',
        nargs='?',
        dest='baud_rate',
        const=DEFAULT_BAUD_RATE,
        default=DEFAULT_BAUD_RATE,
        help='set custom serial baud rate. default: 9600')
    
    # serial port handling
    parser.add_argument('-p',
        nargs='?',
        dest='serial_port',
        const=DEFAULT_SERIAL_PORT,
        default=DEFAULT_SERIAL_PORT,
        help='set serial port name. default: COM3')

    # suppress logging
    parser.add_argument('-D',
        dest='debug_mode',
        action='store_true',
        help='enable debug mode, suppresses file logging')
    
    # set logging level
    parser.add_argument('-L',
        nargs='?',
        dest='logging_level',
        const=DEFAULT_LOGGING_LEVEL,
        default=DEFAULT_LOGGING_LEVEL,
        help='set logging level. default: DEBUG')
        
    return vars(parser.parse_args(argv))

# configures logging
def config_logger(args):  
    # get logging level
    level = args['logging_level'].upper()    
    if level == 'CRITICAL':
        level = logging.CRITICAL
    elif level == 'ERROR':
        level = logging.ERROR
    elif level == 'WARNING':
        level = logging.WARNING
    elif level == 'INFO':
        level = logging.INFO
    else:
        level = logging.DEBUG
    
    # configure logging:
    global logger
    logger = logging.getLogger('commander')
    logger.setLevel(level)

    # create formatter
    formatter = logging.Formatter(fmt='%(levelname)s \t %(asctime)s %(message)s')

    # create console handler
    ch = logging.StreamHandler()
    ch.setLevel(level)
    #ch.setFormatter(formatter) # commented out to avoid cluttering of console
    logger.addHandler(ch)

    if not args['debug_mode']:
        # create file handler
        fh = logging.FileHandler(args['log_filename'])
        fh.setLevel(level)
        fh.setFormatter(formatter)
        logger.addHandler(fh)

if __name__ == '__main__':
    main(parse_arguments(sys.argv[1:]))