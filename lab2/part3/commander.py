from time import sleep
import serial, random, string, sys, logging, argparse
from datetime import datetime

DEFAULT_BAUD_RATE = 9600
DEFAULT_SERIAL_PORT = 'COM3'
DEFAULT_LOGGING_LEVEL = 'DEBUG'

ser = None
logger = None

menu_items = [
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
    global logger
    logger = config_logger(args)

    # Open the serial port
    global ser
    try:
        ser = serial.Serial(args['serial_port'], args['baud_rate'])
    except serial.serialutil.SerialException:
        logger.error('Serial port ' + args['serial_port'] + ' in use or incorrect.')
        logger.info('Program terminated.')
        sys.exit()

    # Wait 2 seconds before sending characters
    sleep(2)

    # Confirm that the connection is established
    if ser.isOpen():
        logger.info('Serial port ' + args['serial_port'] + ' opened successfully.')

    while True:
        print_menu()
        i = input('>>')

        if i == 'q':
            logger.info('Program terminated.')
            break
        elif i.isdigit() and int(i) >= 0 and int(i) < len(menu_items):
            logger.info('Command: ' + str(i) + ' | ' + menu_items[int(i)])

            if i == '0' or i == '1':
                body = input('Payload string >> ')
                logger.info('Sending... ' + i + body)
                serial_send(i + body)
                logger.info('Recieved... ' + serial_read())
            else:
                serial_send(i)
                logger.info('Recieved... ' + serial_read())
        else:
            logger.info('Command not recognized.')

def serial_send(msg):
    ser.write(bytes(msg, 'utf-8'))

def serial_read():
    serial_wait()
    out = ''
    while ser.in_waiting > 0:
        out += ser.read(1).decode('utf-8')
        sleep(0.01)
    return out

def serial_wait():
    # wait for data to come in
    start = datetime.now().second
    while ser.in_waiting == 0:
        if datetime.now().second - start > TIME_OUT:
            logging.getLogger('commander').error('Timeout!!!!')
            break
        pass

def print_menu():
    print('--------------------------------------------------------')
    for i in range(len(menu_items)):
        print('\t' + str(i) + ': ' + menu_items[i])
    print('--------------------------------------------------------')

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
    logger = logging.getLogger('commander')
    logger.setLevel(level)

    # create formatter
    formatter = logging.Formatter(fmt='%(levelname)s \t %(asctime)s %(message)s')

    # create console handler
    ch = logging.StreamHandler()
    ch.setLevel(level)
    #ch.setFormatter(formatter)
    logger.addHandler(ch)
    
    if not args['debug_mode']:
        # create file handler
        fh = logging.FileHandler(args['log_filename'])
        fh.setLevel(level)
        fh.setFormatter(formatter)
        logger.addHandler(fh)

    return logger

if __name__ == '__main__':
    main(parse_arguments(sys.argv[1:]))