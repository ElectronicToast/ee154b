'''
This program tests the fidelity of a serial connection with an Arduino.
It records the transmission error rate of single characters.
'''

from time import sleep
import serial, random, string, sys, logging, argparse, collections
from datetime import datetime

DEFAULT_BAUD_RATE = 9600
DEFAULT_SERIAL_PORT = 'COM10'

NUM_TESTS = 1000

REPITITION_N = 5

def main(args):
    logger = config_logger(args)

    # Open the serial port
    ser = serial.Serial(args['serial_port'], args['baud_rate'])

    # Wait 2 seconds before sending characters
    sleep(2)

    # Confirm that the connection is established
    if ser.isOpen():
        logger.info('Serial port ' + args['serial_port'] + ' opened successfully.')

    # Print test statement
    logger.info('Running: ' + str(NUM_TESTS) + " tests with baud rate: " +
        str(args['baud_rate']))

    # Count errors
    errors = 0.0

    for i in range(NUM_TESTS):
        # send random character
        send = random.choice(string.ascii_letters)
        original = send
        if args['encode']:
            send = encode(send)
        ser.write(bytes(send, 'utf-8'))

        # wait for data to come in
        while ser.in_waiting == 0:
            pass

        # read back echo
        rec = ''
        while ser.in_waiting > 0:
            rec += ser.read(1).decode('utf-8')
        
        original_rec = rec
        
        if args['encode']:
            rec = decode(rec)
        
        if not incremented_correctly(original, rec):
            logger.error(send + " ------> " + original_rec)
            errors += 1
        else:
            logger.debug(send + " ------> " + original_rec)

    logger.info("Error rate: " + str(errors / NUM_TESTS))

def incremented_correctly(orig, inced):
    if orig == 'z':
        return inced == 'a'
    elif orig == 'Z':
        return inced == 'A'
    else:
        return inced == chr(ord(orig) + 1)

def encode(input):
    return input * REPITITION_N

def decode(input):
    return collections.Counter(input).most_common(1)[0][0]
    
def config_logger(args):
    # configure logging:
    logger = logging.getLogger('commander')
    logger.setLevel(logging.DEBUG)
    
    # create formatter
    formatter = logging.Formatter(fmt='%(levelname)s \t %(asctime)s %(message)s')
    
    # create console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    
    # create file handler
    fh = logging.FileHandler(args['log_filename'])
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(formatter)
    logger.addHandler(fh)
    
    return logger
    
def parse_arguments(argv):
    default_filename = datetime.now().strftime("%m-%d-%Y %I-%M-%S %p")+".log"

    parser = argparse.ArgumentParser()

    parser.add_argument('-l',
        nargs='?',
        dest='log_filename',
        const=default_filename,
        default=default_filename,
        help='log filename')

    parser.add_argument('-R',
        nargs='?',
        dest='baud_rate',
        const=DEFAULT_BAUD_RATE,
        default=DEFAULT_BAUD_RATE,
        help='serial baud rate'
        )
    
    parser.add_argument('-e',
        dest='encode',
        action='store_true',
        help='encode or not'
        )

    # serial port handling
    parser.add_argument('-p',
        nargs='?',
        dest='serial_port',
        const=DEFAULT_SERIAL_PORT,
        default=DEFAULT_SERIAL_PORT,
        help='set serial port name. default: COM3')

    return vars(parser.parse_args(argv))

if __name__ == '__main__':
    # parse arguments then pass into main
    main(parse_arguments(sys.argv[1:]))