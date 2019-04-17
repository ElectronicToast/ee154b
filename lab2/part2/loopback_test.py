'''
This program tests the fidelity of a serial connection with an Arduino.
It records the transmission error rate of single characters.
'''

from time import sleep
import serial
import random
import string

BAUDRATE = 9600
PORT = 'COM8'

NUM_TESTS = 2

def main():
    # Open the serial port
    ser = serial.Serial(PORT, BAUDRATE)

    # Confirm that the connection is established
    if ser.isOpen():
        print('Serial port opened successfully.')
    
    errors = 0.0
 
    for i in range(NUM_TESTS):
        # send random character
        rand = random.choice(string.ascii_letters)
        ser.write(bytes(rand, 'utf-8'))
        sleep(0.1)
        
        # read back echo
        out = ''
        while ser.in_waiting > 0:
            out += ser.read(1).decode('utf-8')
    
        # don't print empty reads
        if out != '':
            print(out)

        if out != rand:
            errors += 1
    
    print(errors / NUM_TESTS)
    
if __name__ == '__main__':
    main()


