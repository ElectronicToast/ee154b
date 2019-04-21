'''
This program tests the fidelity of a serial connection with an Arduino.
It records the transmission error rate of single characters.
'''

from time import sleep
import serial, random, string, sys, logging
from datetime import datetime

BAUD_RATE = 4000000
PORT = 'COM7'

NUM_TESTS = 100
SLEEP_TIME = 0.01

REPITITION_N = 5

def main():
    # configure logging:
    logging.basicConfig(
        format='%(asctime)s %(message)s', 
        datefmt='%m/%d/%Y %I:%M:%S %p',
        filename=datetime.now().strftime("%m-%d-%Y %I-%M-%S %p")+".log",
        level=logging.DEBUG)
    
    logging.getLogger().addHandler(logging.StreamHandler())
    

    # Open the serial port
    ser = serial.Serial(PORT, BAUD_RATE)
    sleep(2)
    
    # Confirm that the connection is established
    if ser.isOpen():
        logging.debug('Serial port ' + PORT + ' opened successfully.')

    logging.debug('Running: ' + str(NUM_TESTS) + " tests with baud rate: " + str(BAUD_RATE))

    errors = 0.0

    for i in range(NUM_TESTS):
        # send random character
        rand = random.choice(string.ascii_letters)
        ser.write(bytes(rand, 'utf-8'))
        
        while(ser.in_waiting == 0):
            sleep(0.001)

        # read back echo
        out = ''
        while ser.in_waiting > 0:
            out += ser.read(1).decode('utf-8')

        # don't print empty reads
        if out != '':
            logging.debug(rand + " ------> " + out)
            

        if not incremented_correctly(rand, out):
            errors += 1
            logging.debug('*')

    logging.debug("Error rate: " + str(errors / NUM_TESTS))
    
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
    
        
if __name__ == '__main__':
    main()