'''
This program tests the fidelity of a serial connection with an Arduino.
It records the transmission error rate of single characters.
'''

from time import sleep
import serial, random, string, sys, logging, argparse
from datetime import datetime

DEFAULT_BAUD_RATE = 9600
PORT = 'COM3'

NUM_TESTS = 1000

REPITITION_N = 5

def main(args):

    # configure logging:
    logging.basicConfig(
        format='%(levelname)s %(asctime)s %(message)s',
        datefmt='%m/%d/%Y %I:%M:%S %p',
        filename=args['log_filename'],
        level=logging.DEBUG)

    logging.getLogger().addHandler(logging.StreamHandler())

    # Open the serial port
    ser = serial.Serial(PORT, args['baud_rate'])

    # Wait 2 seconds before sending characters
    sleep(2)

    # Confirm that the connection is established
    if ser.isOpen():
        logging.info('Serial port ' + PORT + ' opened successfully.')

    # Print test statement
    logging.info('Running: ' + str(NUM_TESTS) + " tests with baud rate: " +
        str(args['baud_rate']))

    # Count errors
    errors = 0.0

    for i in range(NUM_TESTS):
        # send random character
        rand = random.choice(string.ascii_letters)
        ser.write(bytes(rand, 'utf-8'))

        # wait for data to come in
        while ser.in_waiting == 0:
            pass

        # read back echo
        out = ''
        while ser.in_waiting > 0:
            out += ser.read(1).decode('utf-8')

        # if not incremented_correctly(rand, out):
        if i == 2:
            logging.error(rand + " ------> " + out)
            errors += 1
        else:
            logging.debug(rand + " ------> " + out)

    logging.info("Error rate: " + str(errors / NUM_TESTS))

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
    pass

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

    return vars(parser.parse_args(argv))

if __name__ == '__main__':
    # parse arguments then pass into main
    main(parse_arguments(sys.argv[1:]))